#!/usr/bin/env python3
"""
UNM Space Tug -- MuJoCo 6-nozzle RCS trajectory tracking (zero-g).

Guidance / control / allocation are written to be NUMERICALLY IDENTICAL to
the C++ version (cpp/src/main.cpp): same constants, same operation order,
same MuJoCo utility calls. Run both with --headless and diff the CSVs.

Usage
  python spacetug_sim.py                      # interactive viewer
  python spacetug_sim.py --traj fig8          # viewer, autopilot pre-engaged
  python spacetug_sim.py --headless --traj fig8 --duration 120 --csv parity_python.csv

Viewer keys:  1/2/3 engage FIG-8 / ORBIT / VERT-8,  0 disengage,  R reset.
"""
import argparse
import math
import os
import time

import numpy as np
import mujoco

# ----------------------------------------------------------------------------
# shared constants (MUST match cpp/src/main.cpp exactly)
# ----------------------------------------------------------------------------
THRUST = 0.26                 # N per nozzle (full throttle)
C30    = 0.8660254037844386   # cos(30 deg), nozzle cant
KP_POS, KD_POS, A_MAX = 3.0, 3.5, 0.34
KP_ATT, KD_ATT        = 3.6, 4.0
INERTIA, MASS         = 0.03, 1.0
R0 = 0.36                     # rendezvous radius [m]
H  = 0.02                     # finite-difference step [rad]
TWO_PI = 2.0 * math.pi

TRAJS = [
    ("fig8",  37.0, lambda s: np.array([1.60*math.sin(s), 0.36*math.cos(2*s), 0.92*math.sin(2*s)])),
    ("orbit", 28.0, lambda s: np.array([1.44*math.cos(s), 0.52*math.sin(s), 1.44*math.sin(s)])),
    ("vert8", 34.0, lambda s: np.array([0.76*math.sin(2*s), 1.16*math.sin(s), 0.40*math.cos(s)])),
]
TRAJ_INDEX = {name: i for i, (name, _, _) in enumerate(TRAJS)}


# ----------------------------------------------------------------------------
# autopilot: PD + feedforward guidance, analytic 6-nozzle allocation,
# attitude PD + tangent-rate feedforward (reaction wheels via xfrc_applied)
# ----------------------------------------------------------------------------
class Autopilot:
    def __init__(self, model, data):
        self.m, self.d = model, data
        self.bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tug")
        self.reset()

    def reset(self):
        self.traj = -1
        self.s = 0.0
        self.err = 0.0
        self.p0 = np.zeros(3)
        self.refUp = np.array([0.0, 1.0, 0.0])
        self.d.ctrl[:] = 0.0
        self.d.xfrc_applied[self.bid, :] = 0.0

    def engage(self, k):
        self.traj = k
        _, _, f = TRAJS[k]
        best, bd = 0.0, 1e18
        p = self.d.qpos[0:3]
        for i in range(128):
            ss = TWO_PI * i / 128.0
            q = f(ss)
            dx, dy, dz = q[0]-p[0], q[1]-p[1], q[2]-p[2]
            di = math.sqrt(dx*dx + dy*dy + dz*dz)
            if di < bd:
                bd, best = di, ss
        self.s = best
        self.err = 0.0
        self.refUp = np.array([0.0, 1.0, 0.0])

    def disengage(self):
        self.traj = -1
        self.d.ctrl[:] = 0.0
        self.d.xfrc_applied[self.bid, 3:6] = 0.0

    def step(self):
        d, m = self.d, self.m
        if self.traj < 0:
            d.ctrl[:] = 0.0
            d.xfrc_applied[self.bid, 3:6] = 0.0
            return

        _, T, f = TRAJS[self.traj]
        dt = m.opt.timestep
        sd = TWO_PI / T

        # --- guidance: rendezvous-scaled reference advance -------------------
        self.s += sd * dt * min(1.0, R0 / max(self.err, R0))
        p0 = f(self.s)
        pa = f(self.s - H)
        pb = f(self.s + H)
        vr  = (pb - pa) * (sd / (2.0 * H))                  # reference velocity
        aff = (pb + pa - 2.0 * p0) * (sd * sd / (H * H))    # feedforward accel
        self.p0 = p0

        # --- state ------------------------------------------------------------
        p = d.qpos[0:3]
        q = d.qpos[3:7]
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.bid, vel, 0)
        w, v = vel[0:3], vel[3:6]                            # world frame

        # --- translation: PD + FF, saturated ----------------------------------
        e = p0 - p
        self.err = math.sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2])
        ad = aff + KP_POS * e + KD_POS * (vr - v)
        n = math.sqrt(ad[0]*ad[0] + ad[1]*ad[1] + ad[2]*ad[2])
        if n > A_MAX:
            ad *= A_MAX / n

        # --- analytic allocation to 6 unidirectional nozzles (body frame) ----
        qi = np.zeros(4); mujoco.mju_negQuat(qi, q)
        Fw = MASS * ad
        Fb = np.zeros(3); mujoco.mju_rotVecQuat(Fb, Fw, qi)
        u = np.zeros(6)
        if Fb[0] > 0: u[0] =  Fb[0] / (C30 * THRUST)
        else:         u[1] = -Fb[0] / (C30 * THRUST)
        if Fb[2] > 0: u[2] =  Fb[2] / (C30 * THRUST)
        else:         u[3] = -Fb[2] / (C30 * THRUST)
        yrem = Fb[1] - 0.5 * THRUST * (u[0] + u[1] + u[2] + u[3])
        if yrem > 0:
            dd = yrem / (2.0 * THRUST)
            u[0] += dd; u[1] += dd; u[2] += dd; u[3] += dd
        else:
            u[4] = u[5] = -yrem / (2.0 * THRUST)
        np.clip(u, 0.0, 1.0, out=u)
        d.ctrl[:] = u

        # --- attitude: nose (-y body, the coneless face) leads the tangent ---
        tau = np.zeros(3)
        vlen = math.sqrt(vr[0]*vr[0] + vr[1]*vr[1] + vr[2]*vr[2])
        if vlen > 1e-4:
            tan = vr / vlen
            rgt = np.cross(self.refUp, tan)
            if rgt[0]*rgt[0] + rgt[1]*rgt[1] + rgt[2]*rgt[2] < 1e-12:
                rgt = np.cross(np.array([tan[1], tan[2], tan[0]]), tan)
            rgt = rgt / math.sqrt(rgt[0]*rgt[0] + rgt[1]*rgt[1] + rgt[2]*rgt[2])
            up2 = np.cross(tan, rgt)
            up2 = up2 / math.sqrt(up2[0]*up2[0] + up2[1]*up2[1] + up2[2]*up2[2])
            self.refUp = up2                                  # parallel transport
            mat = np.array([rgt[0], -tan[0], up2[0],
                            rgt[1], -tan[1], up2[1],
                            rgt[2], -tan[2], up2[2]])         # columns: x,-tan,up
            qt = np.zeros(4); mujoco.mju_mat2Quat(qt, mat)
            qe = np.zeros(4); mujoco.mju_mulQuat(qe, qt, qi)  # error quat (world)
            if qe[0] < 0: qe = -qe
            rotvec = np.zeros(3); mujoco.mju_quat2Vel(rotvec, qe, 1.0)
            adot = aff[0]*tan[0] + aff[1]*tan[1] + aff[2]*tan[2]
            tdot = (aff - adot * tan) / vlen
            wff = np.cross(tan, tdot)                         # omega_ff = t x dt/dt
            tau = INERTIA * (KP_ATT * rotvec + KD_ATT * (wff - w))
        d.xfrc_applied[self.bid, 3:6] = tau


# ----------------------------------------------------------------------------
# viewer overlays: trajectory line, target marker, throttle-scaled plumes
# ----------------------------------------------------------------------------
AMBER = np.array([1.0, 0.71, 0.33, 0.95], dtype=np.float32)
DIM   = np.array([1.0, 0.71, 0.33, 0.40], dtype=np.float32)
PLUME_OUTER = np.array([1.0, 0.42, 0.08, 0.28], dtype=np.float32)
PLUME_MID   = np.array([1.0, 0.80, 0.22, 0.58], dtype=np.float32)
PLUME_CORE  = np.array([1.0, 0.96, 0.74, 0.92], dtype=np.float32)
PLUME_BLUE  = np.array([0.30, 0.72, 1.0, 0.62], dtype=np.float32)
_EYE9 = np.eye(3).flatten()

_PATH_CACHE = {}
def _path_points(k):
    if k not in _PATH_CACHE:
        _, _, f = TRAJS[k]
        _PATH_CACHE[k] = [f(TWO_PI * i / 100.0) for i in range(101)]
    return _PATH_CACHE[k]

def _next_geom(scn):
    if scn.ngeom >= scn.maxgeom:
        return None
    g = scn.geoms[scn.ngeom]
    scn.ngeom += 1
    return g

def _capsule(scn, p0, p1, radius, rgba):
    g = _next_geom(scn)
    if g is None:
        return False
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.zeros(3), np.zeros(3), _EYE9, rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, p0, p1)
    return True

def _sphere(scn, pos, radius, rgba):
    g = _next_geom(scn)
    if g is None:
        return False
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0, 0]), pos, _EYE9, rgba)
    return True

def _draw_plume(scn, data, site_id, nozzle_index, throttle):
    sp = data.site_xpos[site_id]
    xm = data.site_xmat[site_id]
    ex = np.array([-xm[2], -xm[5], -xm[8]])              # exhaust = -site z (world)
    flicker = 0.92 + 0.08 * math.sin(70.0 * data.time + 1.9 * nozzle_index)
    length = (0.16 + 0.46 * throttle) * flicker
    base = sp + 0.010 * ex

    if not _capsule(scn, base, sp + length * ex,
                    0.040 + 0.075 * throttle, PLUME_OUTER):
        return
    if not _capsule(scn, base, sp + 0.72 * length * ex,
                    0.022 + 0.040 * throttle, PLUME_MID):
        return
    if not _capsule(scn, base, sp + 0.46 * length * ex,
                    0.010 + 0.020 * throttle, PLUME_CORE):
        return
    if not _sphere(scn, sp + 0.018 * ex, 0.018 + 0.030 * throttle, PLUME_BLUE):
        return
    _sphere(scn, sp + 0.88 * length * ex, 0.030 + 0.060 * throttle, PLUME_OUTER)

def draw_overlays(scn, model, data, ap, sids):
    scn.ngeom = 0
    if ap.traj >= 0:
        pts = _path_points(ap.traj)
        for i in range(100):
            g = _next_geom(scn)
            if g is None: return
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE,
                                np.zeros(3), np.zeros(3), _EYE9, DIM)
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, 2.0, pts[i], pts[i+1])
        g = _next_geom(scn)
        if g is not None:
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([0.024, 0, 0]), ap.p0, _EYE9, AMBER)
    for i in range(6):
        u = float(data.ctrl[i])
        if u < 0.04: continue
        _draw_plume(scn, data, sids[i], i, u)


# ----------------------------------------------------------------------------
def run_headless(model, data, ap, traj, duration, csv_path):
    ap.engage(TRAJ_INDEX[traj])
    rows = ["t,err,px,py,pz,qw,qx,qy,qz,u1,u2,u3,u4,u5,u6"]
    k = 0
    n_steps = int(round(duration / model.opt.timestep))
    max_err_after_lap = 0.0
    lap_steps = int(round(TRAJS[ap.traj][1] / model.opt.timestep))
    while k < n_steps:
        ap.step()
        mujoco.mj_step(model, data)
        k += 1
        if k % 50 == 0:                                  # every 0.1 s
            q = data.qpos
            u = data.ctrl
            rows.append(",".join("%.9f" % x for x in
                [data.time, ap.err, q[0], q[1], q[2], q[3], q[4], q[5], q[6],
                 u[0], u[1], u[2], u[3], u[4], u[5]]))
        if k > lap_steps:
            max_err_after_lap = max(max_err_after_lap, ap.err)
    with open(csv_path, "w") as fh:
        fh.write("\n".join(rows) + "\n")
    print(f"[python] {traj}: {duration:.0f} s simulated, "
          f"steady-state max err = {max_err_after_lap*1000:.2f} mm -> {csv_path}")


def run_viewer(model, data, ap, sids, traj0):
    import mujoco.viewer
    pending = []

    def kb(key):
        pending.append(key)

    if traj0 is not None:
        ap.engage(TRAJ_INDEX[traj0])

    with mujoco.viewer.launch_passive(model, data, key_callback=kb) as v:
        v.cam.distance = 6.0
        v.cam.elevation = -18
        v.cam.azimuth = 130
        t0 = time.monotonic()
        offset = 0.0
        last_print = 0.0
        while v.is_running():
            while pending:
                key = pending.pop(0)
                if key in (ord('1'), ord('2'), ord('3')):
                    ap.engage(key - ord('1'))
                elif key == ord('0'):
                    ap.disengage()
                elif key == ord('R'):
                    mujoco.mj_resetData(model, data)
                    ap.reset()
                    offset = time.monotonic() - t0       # re-sync clock
            wall = time.monotonic() - t0 - offset
            substeps = 0
            while data.time < wall and substeps < 2000:
                ap.step()
                mujoco.mj_step(model, data)
                substeps += 1
            draw_overlays(v.user_scn, model, data, ap, sids)
            v.sync()
            if ap.traj >= 0 and data.time - last_print > 1.0:
                last_print = data.time
                print(f"t={data.time:7.2f}s  traj={TRAJS[ap.traj][0]:6s}  "
                      f"err={ap.err*1000:7.2f} mm  u={np.array2string(np.asarray(data.ctrl), precision=2)}")
            time.sleep(0.004)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_model = os.path.join(here, "..", "model", "spacetug.xml")
    ap_ = argparse.ArgumentParser(description=__doc__)
    ap_.add_argument("--model", default=default_model)
    ap_.add_argument("--traj", choices=list(TRAJ_INDEX), default=None)
    ap_.add_argument("--headless", action="store_true")
    ap_.add_argument("--duration", type=float, default=120.0)
    ap_.add_argument("--csv", default="parity_python.csv")
    args = ap_.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    ap = Autopilot(model, data)
    sids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr{i+1}")
            for i in range(6)]

    if args.headless:
        run_headless(model, data, ap, args.traj or "fig8", args.duration, args.csv)
    else:
        run_viewer(model, data, ap, sids, args.traj)


if __name__ == "__main__":
    main()
