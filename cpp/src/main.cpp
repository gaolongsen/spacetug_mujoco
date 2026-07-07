// UNM Space Tug -- MuJoCo 6-nozzle RCS trajectory tracking (zero-g), C++.
//
// The guidance / control / allocation code below is written to be NUMERICALLY
// IDENTICAL to python/spacetug_sim.py: same constants, same operation order,
// same MuJoCo utility calls. Run both with --headless and diff the CSVs.
//
// Usage:
//   ./spacetug [model.xml]                                  interactive viewer
//   ./spacetug [model.xml] --traj fig8                      viewer, pre-engaged
//   ./spacetug [model.xml] --headless fig8 120 parity_cpp.csv
//
// Viewer keys: 1/2/3 engage FIG-8 / ORBIT / VERT-8, 0 disengage, R reset.
// Mouse: left-drag rotate, right-drag pan, scroll zoom.

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

namespace tug {

// --------------------------------------------------------------------------
// shared constants (MUST match python/spacetug_sim.py exactly)
// --------------------------------------------------------------------------
constexpr double THRUST = 0.26;                  // N per nozzle (full throttle)
constexpr double C30    = 0.8660254037844386;    // cos(30 deg), nozzle cant
constexpr double KP_POS = 3.0, KD_POS = 3.5, A_MAX = 0.34;
constexpr double KP_ATT = 3.6, KD_ATT = 4.0;
constexpr double INERTIA = 0.03, MASS = 1.0;
constexpr double R0 = 0.36;                      // rendezvous radius [m]
constexpr double H  = 0.02;                      // finite-difference step [rad]
constexpr double TWO_PI = 6.283185307179586;

struct Traj { const char* name; double T; void (*f)(double, double*); };
inline void fig8 (double s, double* o){ o[0]=1.60*std::sin(s); o[1]=0.36*std::cos(2*s); o[2]=0.92*std::sin(2*s); }
inline void orbit(double s, double* o){ o[0]=1.44*std::cos(s); o[1]=0.52*std::sin(s); o[2]=1.44*std::sin(s); }
inline void vert8(double s, double* o){ o[0]=0.76*std::sin(2*s); o[1]=1.16*std::sin(s); o[2]=0.40*std::cos(s); }
inline const Traj TRAJS[3] = {{"fig8",37.0,fig8},{"orbit",28.0,orbit},{"vert8",34.0,vert8}};

inline double dot3(const double* a, const double* b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline double norm3(const double* a){ return std::sqrt(dot3(a,a)); }
inline void cross3(double* r, const double* a, const double* b){
  r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0];
}

// --------------------------------------------------------------------------
// autopilot: PD + feedforward guidance, analytic 6-nozzle allocation,
// attitude PD + tangent-rate feedforward (reaction wheels via xfrc_applied)
// --------------------------------------------------------------------------
struct Autopilot {
  const mjModel* m = nullptr;
  mjData* d = nullptr;
  int bid = -1;
  int traj = -1;
  double s = 0, err = 0;
  double p0[3] = {0,0,0};
  double refUp[3] = {0,1,0};

  void init(const mjModel* m_, mjData* d_) {
    m = m_; d = d_;
    bid = mj_name2id(m, mjOBJ_BODY, "tug");
    reset();
  }
  void reset() {
    traj = -1; s = 0; err = 0;
    refUp[0]=0; refUp[1]=1; refUp[2]=0;
    for (int i = 0; i < m->nu; i++) d->ctrl[i] = 0;
    for (int i = 0; i < 6; i++) d->xfrc_applied[6*bid+i] = 0;
  }
  void engage(int k) {
    traj = k;
    const Traj& t = TRAJS[k];
    double best = 0, bd = 1e18, q[3];
    for (int i = 0; i < 128; i++) {
      double ss = TWO_PI * i / 128.0;
      t.f(ss, q);
      double dx = q[0]-d->qpos[0], dy = q[1]-d->qpos[1], dz = q[2]-d->qpos[2];
      double di = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (di < bd) { bd = di; best = ss; }
    }
    s = best; err = 0;
    refUp[0]=0; refUp[1]=1; refUp[2]=0;
  }
  void disengage() {
    traj = -1;
    for (int i = 0; i < m->nu; i++) d->ctrl[i] = 0;
    for (int i = 0; i < 3; i++) d->xfrc_applied[6*bid+3+i] = 0;
  }

  void step() {
    if (traj < 0) {
      for (int i = 0; i < m->nu; i++) d->ctrl[i] = 0;
      for (int i = 0; i < 3; i++) d->xfrc_applied[6*bid+3+i] = 0;
      return;
    }
    const Traj& t = TRAJS[traj];
    const double dt = m->opt.timestep;
    const double sd = TWO_PI / t.T;

    // --- guidance: rendezvous-scaled reference advance ---------------------
    s += sd * dt * std::min(1.0, R0 / std::max(err, R0));
    double pa[3], pb[3], vr[3], aff[3];
    t.f(s, p0); t.f(s - H, pa); t.f(s + H, pb);
    for (int i = 0; i < 3; i++) {
      vr[i]  = (pb[i] - pa[i]) * (sd / (2.0 * H));            // reference velocity
      aff[i] = (pb[i] + pa[i] - 2.0*p0[i]) * (sd*sd / (H*H)); // feedforward accel
    }

    // --- state --------------------------------------------------------------
    const double* p = d->qpos;
    const double* q = d->qpos + 3;
    double vel[6];
    mj_objectVelocity(m, d, mjOBJ_BODY, bid, vel, 0);
    const double* w = vel;          // angular, world
    const double* v = vel + 3;      // linear, world

    // --- translation: PD + FF, saturated ------------------------------------
    double e[3] = {p0[0]-p[0], p0[1]-p[1], p0[2]-p[2]};
    err = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    double ad[3];
    for (int i = 0; i < 3; i++) ad[i] = aff[i] + KP_POS*e[i] + KD_POS*(vr[i]-v[i]);
    double n = std::sqrt(ad[0]*ad[0] + ad[1]*ad[1] + ad[2]*ad[2]);
    if (n > A_MAX) for (int i = 0; i < 3; i++) ad[i] *= A_MAX / n;

    // --- analytic allocation to 6 unidirectional nozzles (body frame) -------
    double qi[4]; mju_negQuat(qi, q);
    double Fw[3] = {MASS*ad[0], MASS*ad[1], MASS*ad[2]};
    double Fb[3]; mju_rotVecQuat(Fb, Fw, qi);
    double u[6] = {0,0,0,0,0,0};
    if (Fb[0] > 0) u[0] =  Fb[0] / (C30*THRUST); else u[1] = -Fb[0] / (C30*THRUST);
    if (Fb[2] > 0) u[2] =  Fb[2] / (C30*THRUST); else u[3] = -Fb[2] / (C30*THRUST);
    double yrem = Fb[1] - 0.5*THRUST*(u[0]+u[1]+u[2]+u[3]);
    if (yrem > 0) { double dd = yrem / (2.0*THRUST); u[0]+=dd; u[1]+=dd; u[2]+=dd; u[3]+=dd; }
    else          { u[4] = u[5] = -yrem / (2.0*THRUST); }
    for (int i = 0; i < 6; i++) d->ctrl[i] = std::min(1.0, std::max(0.0, u[i]));

    // --- attitude: nose (-y body, the coneless face) leads the tangent ------
    double tau[3] = {0,0,0};
    double vlen = std::sqrt(vr[0]*vr[0] + vr[1]*vr[1] + vr[2]*vr[2]);
    if (vlen > 1e-4) {
      double tan_[3] = {vr[0]/vlen, vr[1]/vlen, vr[2]/vlen};
      double rgt[3]; cross3(rgt, refUp, tan_);
      if (dot3(rgt, rgt) < 1e-12) {
        double tmp[3] = {tan_[1], tan_[2], tan_[0]};
        cross3(rgt, tmp, tan_);
      }
      double rl = std::sqrt(rgt[0]*rgt[0] + rgt[1]*rgt[1] + rgt[2]*rgt[2]);
      for (int i = 0; i < 3; i++) rgt[i] /= rl;
      double up2[3]; cross3(up2, tan_, rgt);
      double ul = std::sqrt(up2[0]*up2[0] + up2[1]*up2[1] + up2[2]*up2[2]);
      for (int i = 0; i < 3; i++) up2[i] /= ul;
      for (int i = 0; i < 3; i++) refUp[i] = up2[i];          // parallel transport
      double mat[9] = {rgt[0], -tan_[0], up2[0],
                       rgt[1], -tan_[1], up2[1],
                       rgt[2], -tan_[2], up2[2]};             // columns: x,-tan,up
      double qt[4]; mju_mat2Quat(qt, mat);
      double qe[4]; mju_mulQuat(qe, qt, qi);                  // error quat (world)
      if (qe[0] < 0) for (int i = 0; i < 4; i++) qe[i] = -qe[i];
      double rotvec[3]; mju_quat2Vel(rotvec, qe, 1.0);
      double adot = dot3(aff, tan_);
      double tdot[3];
      for (int i = 0; i < 3; i++) tdot[i] = (aff[i] - adot*tan_[i]) / vlen;
      double wff[3]; cross3(wff, tan_, tdot);                 // omega_ff = t x dt/dt
      for (int i = 0; i < 3; i++) tau[i] = INERTIA*(KP_ATT*rotvec[i] + KD_ATT*(wff[i]-w[i]));
    }
    for (int i = 0; i < 3; i++) d->xfrc_applied[6*bid+3+i] = tau[i];
  }
};

}  // namespace tug

// --------------------------------------------------------------------------
// headless parity run
// --------------------------------------------------------------------------
static int run_headless(const mjModel* m, mjData* d, tug::Autopilot& ap,
                        const std::string& traj, double duration,
                        const std::string& csv) {
  int k_traj = -1;
  for (int i = 0; i < 3; i++) if (traj == tug::TRAJS[i].name) k_traj = i;
  if (k_traj < 0) { std::fprintf(stderr, "unknown trajectory '%s'\n", traj.c_str()); return 1; }
  ap.engage(k_traj);

  FILE* fh = std::fopen(csv.c_str(), "w");
  if (!fh) { std::fprintf(stderr, "cannot open %s\n", csv.c_str()); return 1; }
  std::fprintf(fh, "t,err,px,py,pz,qw,qx,qy,qz,u1,u2,u3,u4,u5,u6\n");

  long n_steps = std::lround(duration / m->opt.timestep);
  long lap_steps = std::lround(tug::TRAJS[k_traj].T / m->opt.timestep);
  double max_err_after_lap = 0.0;
  for (long k = 1; k <= n_steps; k++) {
    ap.step();
    mj_step(m, d);
    if (k % 50 == 0) {
      const double* q = d->qpos;
      const double* u = d->ctrl;
      std::fprintf(fh,
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
        d->time, ap.err, q[0], q[1], q[2], q[3], q[4], q[5], q[6],
        u[0], u[1], u[2], u[3], u[4], u[5]);
    }
    if (k > lap_steps) max_err_after_lap = std::max(max_err_after_lap, ap.err);
  }
  std::fclose(fh);
  std::printf("[cpp] %s: %.0f s simulated, steady-state max err = %.2f mm -> %s\n",
              traj.c_str(), duration, max_err_after_lap*1000.0, csv.c_str());
  return 0;
}

// --------------------------------------------------------------------------
// interactive viewer (GLFW)
// --------------------------------------------------------------------------
#ifdef HAVE_GLFW
namespace ui {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  tug::Autopilot* ap = nullptr;
  mjvCamera cam; mjvOption opt; mjvScene scn; mjrContext con; mjvPerturb pert;
  bool bl = false, bm = false, br = false;
  double lx = 0, ly = 0;

  bool ctrl_down(GLFWwindow* w) {
    return glfwGetKey(w, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
           glfwGetKey(w, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
  }

  void keyboard(GLFWwindow*, int key, int, int act, int) {
    if (act != GLFW_PRESS) return;
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_3) ap->engage(key - GLFW_KEY_1);
    else if (key == GLFW_KEY_0) ap->disengage();
    else if (key == GLFW_KEY_R || key == GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d); ap->reset(); glfwSetTime(0);
    }
  }
  void mouse_button(GLFWwindow* w, int, int, int) {
    bl = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    bm = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    br = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(w, &lx, &ly);

    pert.active = 0;
    pert.active2 = 0;
    if (!ctrl_down(w) || (!bl && !br)) return;

    int wd, ht;
    glfwGetWindowSize(w, &wd, &ht);
    if (wd <= 0 || ht <= 0) return;

    double selpnt[3] = {0, 0, 0};
    int geomid = -1, flexid = -1, skinid = -1;
    int bodyid = mjv_select(m, d, &opt, static_cast<double>(wd) / ht,
                            lx / wd, (ht - ly) / ht, &scn, selpnt,
                            &geomid, &flexid, &skinid);
    if (bodyid > 0) {
      pert.select = bodyid;
      pert.flexselect = flexid;
      pert.skinselect = skinid;
      mjv_initPerturb(m, d, &scn, &pert);
    } else {
      pert.select = 0;
    }

    if (pert.select > 0) {
      pert.active = bl ? mjPERT_TRANSLATE : mjPERT_ROTATE;
    }
  }
  void mouse_move(GLFWwindow* w, double x, double y) {
    if (!bl && !bm && !br) return;
    double dx = x - lx, dy = y - ly; lx = x; ly = y;
    int wd, ht; glfwGetWindowSize(w, &wd, &ht);
    if (ht <= 0) return;
    bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                 glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    if (!ctrl_down(w)) {
      pert.active = 0;
      pert.active2 = 0;
    }

    if (ctrl_down(w) && pert.select > 0 && (bl || br)) {
      mjtMouse action = bl ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                           : (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
      mjv_movePerturb(m, d, action, dx/ht, dy/ht, &scn, &pert);
      return;
    }

    mjtMouse action = bl ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                    : br ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                         : mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx/ht, dy/ht, &scn, &cam);
  }
  void scroll(GLFWwindow*, double, double yoff) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoff, &scn, &cam);
  }

  void add_overlays(const int sid[6]) {
    auto next = [&]() -> mjvGeom* {
      if (scn.ngeom >= scn.maxgeom) return nullptr;
      return scn.geoms + scn.ngeom++;
    };
    const float amber[4] = {1.0f, 0.71f, 0.33f, 0.95f};
    const float dim[4]   = {1.0f, 0.71f, 0.33f, 0.40f};
    const float plume_outer[4] = {1.0f, 0.42f, 0.08f, 0.28f};
    const float plume_mid[4]   = {1.0f, 0.80f, 0.22f, 0.58f};
    const float plume_core[4]  = {1.0f, 0.96f, 0.74f, 0.92f};
    const float plume_blue[4]  = {0.30f, 0.72f, 1.0f, 0.62f};
    if (ap->traj >= 0) {
      static std::vector<double> pts(101*3);
      for (int i = 0; i <= 100; i++)
        tug::TRAJS[ap->traj].f(tug::TWO_PI*i/100.0, &pts[3*i]);
      for (int i = 0; i < 100; i++) {
        mjvGeom* g = next(); if (!g) return;
        mjv_initGeom(g, mjGEOM_LINE, nullptr, nullptr, nullptr, dim);
        mjv_connector(g, mjGEOM_LINE, 2.0, &pts[3*i], &pts[3*i+3]);
      }
      mjvGeom* g = next();
      if (g) {
        double size[3] = {0.024, 0, 0};
        mjv_initGeom(g, mjGEOM_SPHERE, size, ap->p0, nullptr, amber);
      }
    }

    auto add_plume = [&](int i, double u) {
      const double* sp = d->site_xpos + 3*sid[i];
      const double* xm = d->site_xmat + 9*sid[i];
      double ex[3] = {-xm[2], -xm[5], -xm[8]};        // exhaust = -site z (world)
      double flicker = 0.92 + 0.08*std::sin(70.0*d->time + 1.9*i);
      double L = (0.16 + 0.46*u) * flicker;
      double base[3] = {sp[0] + 0.010*ex[0], sp[1] + 0.010*ex[1], sp[2] + 0.010*ex[2]};
      double outer_tip[3] = {sp[0] + L*ex[0], sp[1] + L*ex[1], sp[2] + L*ex[2]};
      double mid_tip[3] = {sp[0] + 0.72*L*ex[0], sp[1] + 0.72*L*ex[1], sp[2] + 0.72*L*ex[2]};
      double core_tip[3] = {sp[0] + 0.46*L*ex[0], sp[1] + 0.46*L*ex[1], sp[2] + 0.46*L*ex[2]};

      mjvGeom* g = next(); if (!g) return;
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, plume_outer);
      mjv_connector(g, mjGEOM_CAPSULE, 0.040 + 0.075*u, base, outer_tip);

      g = next(); if (!g) return;
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, plume_mid);
      mjv_connector(g, mjGEOM_CAPSULE, 0.022 + 0.040*u, base, mid_tip);

      g = next(); if (!g) return;
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, plume_core);
      mjv_connector(g, mjGEOM_CAPSULE, 0.010 + 0.020*u, base, core_tip);

      g = next(); if (!g) return;
      double flare_pos[3] = {sp[0] + 0.018*ex[0], sp[1] + 0.018*ex[1], sp[2] + 0.018*ex[2]};
      double flare_size[3] = {0.018 + 0.030*u, 0, 0};
      mjv_initGeom(g, mjGEOM_SPHERE, flare_size, flare_pos, nullptr, plume_blue);

      g = next(); if (!g) return;
      double glow_pos[3] = {sp[0] + 0.88*L*ex[0], sp[1] + 0.88*L*ex[1], sp[2] + 0.88*L*ex[2]};
      double glow_size[3] = {0.030 + 0.060*u, 0, 0};
      mjv_initGeom(g, mjGEOM_SPHERE, glow_size, glow_pos, nullptr, plume_outer);
    };

    for (int i = 0; i < 6; i++) {
      double u = d->ctrl[i];
      if (u < 0.04) continue;
      add_plume(i, u);
    }
  }
}  // namespace ui

static int run_viewer(mjModel* m, mjData* d, tug::Autopilot& ap,
                      const std::string& traj0) {
  ui::m = m; ui::d = d; ui::ap = &ap;
  if (!traj0.empty())
    for (int i = 0; i < 3; i++)
      if (traj0 == tug::TRAJS[i].name) ap.engage(i);

  if (!glfwInit()) { std::fprintf(stderr, "GLFW init failed\n"); return 1; }
  GLFWwindow* win = glfwCreateWindow(1280, 800, "UNM Space Tug - MuJoCo", nullptr, nullptr);
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  mjv_defaultCamera(&ui::cam);
  mjv_defaultOption(&ui::opt);
  mjv_defaultPerturb(&ui::pert);
  ui::opt.flags[mjVIS_PERTFORCE] = 1;
  ui::opt.flags[mjVIS_PERTOBJ] = 1;
  mjv_defaultScene(&ui::scn);
  mjr_defaultContext(&ui::con);
  mjv_makeScene(m, &ui::scn, 2000);
  mjr_makeContext(m, &ui::con, mjFONTSCALE_150);
  ui::cam.distance = 6.0; ui::cam.elevation = -18; ui::cam.azimuth = 130;

  glfwSetKeyCallback(win, ui::keyboard);
  glfwSetCursorPosCallback(win, ui::mouse_move);
  glfwSetMouseButtonCallback(win, ui::mouse_button);
  glfwSetScrollCallback(win, ui::scroll);

  int sid[6];
  for (int i = 0; i < 6; i++) {
    char nm[8]; std::snprintf(nm, sizeof nm, "thr%d", i+1);
    sid[i] = mj_name2id(m, mjOBJ_SITE, nm);
  }

  glfwSetTime(0);
  while (!glfwWindowShouldClose(win)) {
    double wall = glfwGetTime();
    int sub = 0;
    while (d->time < wall && sub++ < 2000) {
      ap.step();
      if (ui::pert.select > 0 && ui::pert.active) {
        mjv_applyPerturbForce(m, d, &ui::pert);
      } else if (ui::pert.select > 0) {
        for (int i = 0; i < 3; i++) d->xfrc_applied[6*ui::pert.select+i] = 0;
      }
      mj_step(m, d);
    }

    mjrRect view = {0, 0, 0, 0};
    glfwGetFramebufferSize(win, &view.width, &view.height);
    mjv_updateScene(m, d, &ui::opt, &ui::pert, &ui::cam, mjCAT_ALL, &ui::scn);
    ui::add_overlays(sid);
    mjr_render(view, &ui::scn, &ui::con);

    char status[256];
    std::snprintf(status, sizeof status,
        "traj: %s   err: %.1f mm   keys: 1/2/3  0 off  R reset   Ctrl+left/right drag: force/torque",
        ap.traj >= 0 ? tug::TRAJS[ap.traj].name : "none", ap.err*1000.0);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, view, status, nullptr, &ui::con);

    glfwSwapBuffers(win);
    glfwPollEvents();
  }
  mjr_freeContext(&ui::con);
  mjv_freeScene(&ui::scn);
  glfwTerminate();
  return 0;
}
#endif  // HAVE_GLFW

// --------------------------------------------------------------------------
int main(int argc, char** argv) {
  std::string model_path = "model/spacetug.xml";
  std::string traj, csv = "parity_cpp.csv";
  bool headless = false;
  double duration = 120.0;

  int i = 1;
  if (i < argc && argv[i][0] != '-') model_path = argv[i++];
  for (; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--headless") {
      headless = true;
      if (i+1 < argc && argv[i+1][0] != '-') traj = argv[++i];
      if (i+1 < argc && argv[i+1][0] != '-') duration = std::atof(argv[++i]);
      if (i+1 < argc && argv[i+1][0] != '-') csv = argv[++i];
    } else if (a == "--traj" && i+1 < argc) {
      traj = argv[++i];
    }
  }

  char err[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, err, sizeof err);
  if (!m) { std::fprintf(stderr, "model load failed: %s\n", err); return 1; }
  mjData* d = mj_makeData(m);

  tug::Autopilot ap;
  ap.init(m, d);

  int rc;
  if (headless) {
    rc = run_headless(m, d, ap, traj.empty() ? "fig8" : traj, duration, csv);
  } else {
#ifdef HAVE_GLFW
    rc = run_viewer(m, d, ap, traj);
#else
    std::fprintf(stderr, "built without GLFW; only --headless mode available\n");
    rc = 1;
#endif
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return rc;
}
