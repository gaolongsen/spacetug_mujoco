# UNM Space Tug — MuJoCo 6-Nozzle RCS Trajectory Tracking

A free-floating spacecraft (your `tugnew5` STL) tracks 3D reference
trajectories (∞ figure-8, tilted orbit, vertical 8) in zero gravity, using
only its six real nozzles for translation, with throttle-scaled visualized
plumes. Identical Python and C++ implementations — verified bit-identical.

See `ALGORITHM.md` for the full GNC writeup.

```
spacetug_mujoco/
├── ALGORITHM.md              GNC algorithm reference
├── model/spacetug.xml        MJCF: free joint, mesh, 6 site-thruster motors
├── assets/tugnew5_centered.stl
├── python/spacetug_sim.py    Python version (viewer + headless)
├── python/requirements.txt
├── cpp/CMakeLists.txt
├── cpp/src/main.cpp          C++ version (viewer + headless)
└── tools/compare_parity.py   diff the two CSV outputs
```

## Python version

```bash
cd python
pip install -r requirements.txt          # mujoco >= 3.1, numpy
python spacetug_sim.py                   # interactive viewer
python spacetug_sim.py --traj fig8       # autopilot pre-engaged
```

Viewer keys: **1 / 2 / 3** engage FIG-8 / ORBIT / VERT-8, **0** disengage,
**R** reset. Mouse: left-drag rotate, right-drag pan, scroll zoom. The Python
viewer uses MuJoCo's built-in interactive viewer, so its standard body
selection/perturb controls are available. Amber dashed line = reference path,
amber sphere = target point, and layered blue/white/amber plumes show
throttle-scaled exhaust from each active nozzle. Telemetry (tracking error in
mm, all six throttles) prints to the terminal.

## C++ version

Requires a MuJoCo release (https://github.com/google-deepmind/mujoco/releases,
extract anywhere) and optionally GLFW for the interactive window
(`sudo apt install libglfw3-dev` / `brew install glfw`). Without GLFW the
build still works in headless mode. On Linux, choose the release archive that
matches your CPU architecture, e.g. `linux-x86_64` for a normal Intel/AMD PC.

### This PC

This machine is `x86_64` and has MuJoCo 3.10.0 installed here:

```bash
$HOME/.mujoco/mujoco-3.10.0-x86_64
```

Use these commands from the repository root:

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

Run deterministic headless mode:

```bash
./build/spacetug ../model/spacetug.xml --headless fig8 120 parity_cpp.csv
```

### Other Linux PCs

Download the MuJoCo archive that matches your CPU from the MuJoCo releases
page. For most Intel/AMD Ubuntu desktops, use the `linux-x86_64` archive. For
ARM machines, use the `linux-aarch64` archive.

Example:

```bash
mkdir -p "$HOME/.mujoco"
# Replace the filename with the MuJoCo version/architecture you downloaded.
tar -xzf mujoco-3.10.0-linux-x86_64.tar.gz -C "$HOME/.mujoco"

sudo apt install cmake build-essential libglfw3-dev

cd cpp
cmake -B build -DCMAKE_PREFIX_PATH="$HOME/.mujoco/mujoco-3.10.0"
cmake --build build
./build/spacetug ../model/spacetug.xml --traj fig8
```

If CMake cannot find `mujocoConfig.cmake`, this project will still try to find
`include/mujoco/mujoco.h` and `lib/libmujoco.so` directly under
`CMAKE_PREFIX_PATH`. If the linker reports `file in wrong format`, the MuJoCo
archive architecture does not match the PC; download the correct Linux archive.

### C++ viewer controls

- **1 / 2 / 3**: engage FIG-8 / ORBIT / VERT-8
- **0**: disengage autopilot
- **R**: reset simulation
- Left-drag: rotate camera
- Right-drag: pan camera
- Scroll: zoom
- **Ctrl + left-drag** the spacecraft: apply external force
- **Ctrl + right-drag** the spacecraft: apply external torque
- **Shift + drag**: switch the perturb/camera drag plane

## Verifying both versions give the same result

Both binaries have a deterministic headless mode that logs
`t, err, position, quaternion, 6 throttles` at 10 Hz:

```bash
python python/spacetug_sim.py --headless --traj fig8 --duration 120 --csv parity_python.csv
./cpp/build/spacetug model/spacetug.xml --headless fig8 120 parity_cpp.csv
python tools/compare_parity.py parity_python.csv parity_cpp.csv
```

Expected output: max difference `0.000e+00` on every column (`PARITY OK`).
Both implementations share constants, operation order, and MuJoCo's own
`mju_*` quaternion utilities, and MuJoCo's RK4 integrator is deterministic,
so the trajectories agree to the last bit.

## Notes

- Coordinates match the original web demo: the craft's nozzle-free face is
  −y (the nose); T5/T6 on +y are the aft main engines. Gravity is off, so
  the viewer's z-up convention is cosmetic.
- Thruster forces are applied by MuJoCo at the true nozzle exit sites, so
  parasitic torques r × F are real; attitude is held by reaction-wheel
  torques (`xfrc_applied`), as on real spacecraft.
- Tested with MuJoCo 3.9.0. Anything ≥ 3.1 should work (`mjv_connector`
  is used for the overlay lines/plumes).
