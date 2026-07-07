# Space Tug GNC — Algorithm Reference

Guidance, control, and allocation for a free-floating 6-nozzle spacecraft
tracking 3D reference trajectories in zero gravity. This document is the
authoritative spec; `python/spacetug_sim.py` and `cpp/src/main.cpp` implement
it line-for-line.

## 1. Vehicle and actuator geometry

Nozzle exit positions and exhaust axes were extracted from the STL itself:
each nozzle's flat annular end-cap is a cluster of coplanar triangles, so its
shared normal gives the exhaust axis and its centroid gives the exit center.
The face with no nozzle (−y) is the nose.

| Nozzle | exit position (m, body) | exhaust direction | thrust direction |
|---|---|---|---|
| T1 | (−0.2160, −0.1137, 0) | (−c, −½, 0) | (+c, +½, 0) |
| T2 | (+0.2164, −0.1294, 0) | (+c, −½, 0) | (−c, +½, 0) |
| T3 | (+0.0136, +0.0062, −0.2029) | (0, −½, −c) | (0, +½, +c) |
| T4 | (+0.0056, −0.0027, +0.2032) | (0, −½, +c) | (0, +½, −c) |
| T5 | (−0.0805, +0.2250, 0) | (0, +1, 0) | (0, −1, 0) |
| T6 | (+0.0855, +0.2250, 0) | (0, +1, 0) | (0, −1, 0) |

c = cos 30° = 0.8660254. Per-nozzle thrust `T_n = 0.26 N`, throttle u ∈ [0, 1].
Mass 1 kg, inertia 0.03 kg·m² (isotropic). All quantities are the web-demo
values scaled by S = 0.01 (mm → m), which leaves every gain unchanged.

The thrust directions positively span R³: ±x from T1/T2, ±z from T3/T4,
+y from any canted nozzle, −y from T5/T6 — which is what makes the analytic
allocation in §4 exact.

## 2. Guidance

A reference point moves along the chosen curve p_ref(s), s = phase:

- FIG-8: (1.60 sin s, 0.36 cos 2s, 0.92 sin 2s), period T = 37 s
- ORBIT: (1.44 cos s, 0.52 sin s, 1.44 sin s), T = 28 s
- VERT-8: (0.76 sin 2s, 1.16 sin s, 0.40 cos s), T = 34 s

Reference velocity and feedforward acceleration come from central finite
differences with h = 0.02 rad:

    v_r = (p(s+h) − p(s−h)) · ṡ / 2h
    a_ff = (p(s+h) + p(s−h) − 2p(s)) · ṡ² / h²

**Rendezvous logic.** On engage, s starts at the curve point nearest the
craft. The phase rate is scaled by `min(1, R0 / max(err, R0))` with
R0 = 0.36 m: while the craft is far, the reference waits; within R0 it runs
at full speed. This bounds the engage transient (lock in ≈ 10 s from any
start) without a separate approach mode.

## 3. Translation control

PD with full feedforward, then saturation:

    a_des = a_ff + Kp (p_ref − p) + Kd (v_r − v),   Kp = 3.0, Kd = 3.5
    |a_des| ≤ A_MAX = 0.34 m/s²

Damping ratio ζ = Kd / (2√Kp) ≈ 1.0. With exact feedforward the PD only
rejects disturbances and discretization error; measured steady-state error in
MuJoCo at 500 Hz control is < 0.1 mm on trajectories spanning 3.2 m.

## 4. Thruster allocation (analytic, exact within limits)

Desired force is rotated into the body frame, F = R⁻¹ m a_des, then solved in
closed form using the structure of the layout:

1. x-channel: F_x > 0 → u₁ = F_x / (c·T_n); else u₂ = −F_x / (c·T_n)
2. z-channel: F_z > 0 → u₃ = F_z / (c·T_n); else u₄ = −F_z / (c·T_n)
3. y-channel: the canted throttles already produce
   Y_c = ½ T_n (u₁+u₂+u₃+u₄). For the remainder y_rem = F_y − Y_c:
   - y_rem > 0: add δ = y_rem / (2 T_n) to **all four** canted nozzles —
     their lateral components cancel pairwise, leaving pure +y;
   - y_rem < 0: u₅ = u₆ = −y_rem / (2 T_n).
4. Clamp every uᵢ to [0, 1].

Within actuator limits, the produced net force equals the request exactly
(unit-tested). Per-axis force limits: |F_x|, |F_z| ≤ c·T_n = 0.225 N,
F_y ∈ [−0.52, +0.52] N.

In MuJoCo each nozzle is a `<motor>` on a site, so forces act at the true
exit positions and the resulting parasitic torques r × F are physically real;
the attitude loop rejects them as disturbances.

## 5. Attitude control

**Target frame.** The nose (body −y, the coneless face) must lead along the
path tangent t̂ = v_r / |v_r|, i.e. velocity stays normal to the front face.
The roll degree of freedom is fixed by a **parallel-transported up vector**:

    right = normalize(refUp × t̂)
    up    = t̂ × right
    refUp ← up        (carried to the next step)

This makes the target orientation C¹-smooth along any regular curve — no
gimbal-style flips when the tangent passes vertical, and minimal roll motion
(verified: < 1° target change per 60 Hz frame on all three curves).
The target rotation has columns (right, −t̂, up).

**Control law.** With error quaternion q_e = q_t ⊗ q⁻¹ (shortest rotation,
converted to rotation vector θ) and measured body rate ω:

    τ = I [ K_att θ + D_att (ω_ff − ω) ],   K_att = 3.6, D_att = 4.0

**Rate feedforward** is the key to corner tracking. The angular velocity the
turning tangent demands is

    dt̂/dt = (a_ff − (a_ff·t̂) t̂) / |v_r|,   ω_ff = t̂ × dt̂/dt

A PD that damps toward zero rate has a steady lag angle ≈ D·ω/K under
sustained rotation — measured 35.8° at the FIG-8 crossing. Damping toward
ω_ff instead reduces it to 2.1°.

Attitude torque is applied through `xfrc_applied` (an internal reaction-wheel
/ CMG package, standard on real spacecraft); the nozzles handle translation
only, since six unidirectional thrusters cannot realize arbitrary
force-plus-torque wrenches simultaneously.

## 6. Verified performance (MuJoCo 3.9, RK4, dt = 2 ms)

| Trajectory | engage → lock | steady-state max error |
|---|---|---|
| FIG-8 | ≈ 10 s | 0.09 mm |
| ORBIT | ≈ 13 s | 0.03 mm |
| VERT-8 | ≈ 10 s | 0.07 mm |

Python and C++ headless runs produce **bit-identical** CSV output
(`tools/compare_parity.py` reports 0.0 max difference on every column for all
three trajectories).
