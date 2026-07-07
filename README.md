# UNM Space Tug — MuJoCo 6-Nozzle RCS Trajectory Tracking

![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-0A7FBF)
![Python](https://img.shields.io/badge/Python-viewer%20%2B%20headless-3776AB)
![C++](https://img.shields.io/badge/C%2B%2B17-viewer%20%2B%20headless-00599C)
![Parity](https://img.shields.io/badge/Python%20%3D%20C%2B%2B-bit--identical-success)

A free-floating spacecraft tracks 3D reference trajectories in zero gravity
using only its six real MuJoCo site-thruster nozzles for translation. The
Python and C++ implementations share the same guidance, control, allocation,
and RK4 simulation path, so their headless CSV outputs match bit-for-bit.

The viewer includes throttle-scaled blue/white/amber exhaust plumes, reference
trajectory overlays, a live tracking-error display, and interactive body
perturbation so you can pull the tug away and watch the controller recover.

📘 Full guidance and control writeup: [ALGORITHM.md](ALGORITHM.md)

## ✨ Highlights

| Feature | Details |
| --- | --- |
| 🛰️ Spacecraft model | `tugnew5` STL, free joint, 1 kg, isotropic inertia |
| 🔥 Propulsion | 6 real site-thruster motors at nozzle exit locations |
| 🎯 Trajectories | FIG-8, tilted orbit, vertical-8 |
| 🧭 Control | PD + feedforward translation, analytic nozzle allocation |
| 🕹️ Interaction | Camera control, trajectory hotkeys, body force/torque perturbation |
| ✅ Verification | Deterministic Python/C++ headless parity CSV comparison |

## 📁 Repository Layout

```text
spacetug_mujoco/
├── ALGORITHM.md              GNC algorithm reference
├── model/spacetug.xml        MJCF: free joint, mesh, 6 site-thruster motors
├── assets/tugnew5_centered.stl
├── python/spacetug_sim.py    Python version: viewer + headless
├── python/requirements.txt   Python dependencies
├── cpp/CMakeLists.txt
├── cpp/src/main.cpp          C++ version: viewer + headless
└── tools/compare_parity.py   Compare Python/C++ CSV logs
```





<img src="https://github.com/gaolongsen/picx-images-hosting/raw/master/tug_mujoco_show.96afdvpyrv.gif" style="zoom:200%;" />





## 🚀 Quick Start

| Goal | Command |
| --- | --- |
| Python viewer | `python python/spacetug_sim.py --traj fig8` |
| C++ viewer on this PC | `cd cpp && ./build/spacetug ../model/spacetug.xml --traj fig8` |
| C++ headless | `./cpp/build/spacetug model/spacetug.xml --headless fig8 120 parity_cpp.csv` |
| Parity check | `python tools/compare_parity.py parity_python.csv parity_cpp.csv` |

Available trajectories:

```text
fig8    orbit    vert8
```

## 🐍 Python Version

Install dependencies:

```bash
pip install -r python/requirements.txt
```

Run the viewer:

```bash
python python/spacetug_sim.py                   # viewer, no autopilot
python python/spacetug_sim.py --traj fig8       # viewer, FIG-8 pre-engaged
python python/spacetug_sim.py --traj orbit      # viewer, orbit pre-engaged
python python/spacetug_sim.py --traj vert8      # viewer, vertical-8 pre-engaged
```

Run headless:

```bash
python python/spacetug_sim.py --headless --traj fig8 --duration 120 --csv parity_python.csv
```

The Python viewer uses MuJoCo's built-in interactive viewer, so standard MuJoCo
camera, selection, and perturb controls are available.

![](https://github.com/gaolongsen/picx-images-hosting/raw/master/8shapetraj.6f1d5tkj1j.gif)

## 🧱 C++ Version

The C++ executable supports both the interactive viewer and deterministic
headless runs. GLFW is required only for the interactive window.

<img src="https://github.com/gaolongsen/picx-images-hosting/raw/master/cpptwoshapetraj.9rk308271o.gif" style="zoom:67%;" />

### ✅ This PC

This machine is `x86_64`. The working MuJoCo installation is:

```bash
$HOME/.mujoco/mujoco-3.10.0-x86_64
```

Build from the repository root:

```bash
cd cpp
cmake -B build -DCMAKE_PREFIX_PATH="$HOME/.mujoco/mujoco-3.10.0-x86_64" -Uglfw3_DIR
cmake --build build
```

Run the interactive viewer:

```bash
./build/spacetug ../model/spacetug.xml                # viewer, no autopilot
./build/spacetug ../model/spacetug.xml --traj fig8    # viewer, FIG-8 pre-engaged
./build/spacetug ../model/spacetug.xml --traj orbit   # viewer, orbit pre-engaged
./build/spacetug ../model/spacetug.xml --traj vert8   # viewer, vertical-8 pre-engaged
```

Run headless:

```bash
./build/spacetug ../model/spacetug.xml --headless fig8 120 parity_cpp.csv
```

### 🌍 Other PCs

1. Download MuJoCo from the official releases page:
   https://github.com/google-deepmind/mujoco/releases

2. Pick the archive matching your CPU:

| Machine | MuJoCo archive |
| --- | --- |
| Intel/AMD Ubuntu desktop | `linux-x86_64` |
| ARM Linux machine | `linux-aarch64` |
| macOS Intel | `macos-x86_64` |
| macOS Apple Silicon | `macos-arm64` |

3. On Ubuntu/Linux, install build tools and GLFW:

```bash
sudo apt install cmake build-essential libglfw3-dev
```

4. Extract MuJoCo and build:

```bash
mkdir -p "$HOME/.mujoco"
tar -xzf mujoco-3.10.0-linux-x86_64.tar.gz -C "$HOME/.mujoco"

cd cpp
cmake -B build -DCMAKE_PREFIX_PATH="$HOME/.mujoco/mujoco-3.10.0"
cmake --build build
./build/spacetug ../model/spacetug.xml --traj fig8
```

If you use a different MuJoCo version, change both the archive name and
`CMAKE_PREFIX_PATH` to match the extracted folder.

### 🛠️ CMake Notes

Some MuJoCo binary releases provide headers and libraries but no
`mujocoConfig.cmake`. This project handles both layouts:

| MuJoCo layout | How the project finds it |
| --- | --- |
| Has `mujocoConfig.cmake` | `find_package(mujoco CONFIG)` |
| Has only `include/` and `lib/` | fallback search under `CMAKE_PREFIX_PATH` |

If the linker reports `file in wrong format`, the MuJoCo archive architecture
does not match the PC. Download the correct release archive.

## 🕹️ Viewer Controls

| Input | Action |
| --- | --- |
| `1` / `2` / `3` | Engage FIG-8 / ORBIT / VERT-8 |
| `0` | Disengage autopilot |
| `R` | Reset simulation |
| Left-drag | Rotate camera |
| Right-drag | Pan camera |
| Scroll | Zoom |
| `Ctrl` + left-drag spacecraft | Apply external force |
| `Ctrl` + right-drag spacecraft | Apply external torque |
| `Shift` + drag | Switch camera/perturb drag plane |

Visual overlays:

| Overlay | Meaning |
| --- | --- |
| Amber dashed curve | Reference trajectory |
| Amber sphere | Current target point |
| Blue/white/amber plumes | Active nozzle exhaust, scaled by throttle |

## ✅ Python/C++ Parity Check

Both binaries have a deterministic headless mode that logs
`t, err, position, quaternion, 6 throttles` at 10 Hz.

```bash
python python/spacetug_sim.py --headless --traj fig8 --duration 120 --csv parity_python.csv
./cpp/build/spacetug model/spacetug.xml --headless fig8 120 parity_cpp.csv
python tools/compare_parity.py parity_python.csv parity_cpp.csv
```

Expected result:

```text
PARITY OK
```

The implementations match because they share constants, operation order,
MuJoCo `mju_*` quaternion utilities, and MuJoCo's deterministic RK4 integrator.

## 🔧 Troubleshooting

| Symptom | Fix |
| --- | --- |
| `built without GLFW; only --headless mode available` | Install `libglfw3-dev`, rerun CMake, rebuild |
| `Could not find mujocoConfig.cmake` | Point `CMAKE_PREFIX_PATH` at the extracted MuJoCo folder |
| `file in wrong format` while linking | Download the MuJoCo archive matching your CPU architecture |
| Viewer opens but no autopilot | Run with `--traj fig8`, `--traj orbit`, or press `1` / `2` / `3` |
| Body perturb does not work in C++ | Hold `Ctrl`, click the spacecraft, then drag |

## 🧩 Model Notes

- Coordinates match the original web demo: the nozzle-free face is `-y`, and
  T5/T6 on `+y` are the aft main engines.
- Gravity is disabled; the viewer's `z`-up convention is cosmetic.
- Thruster forces are applied by MuJoCo at the true nozzle exit sites, so
  parasitic torques `r × F` are physically present.
- Attitude is held by reaction-wheel torques through `xfrc_applied`.
- Tested with MuJoCo 3.9.0 and 3.10.0. Anything `>= 3.1` should work because
  `mjv_connector` is used for overlay lines and plume geometry.
