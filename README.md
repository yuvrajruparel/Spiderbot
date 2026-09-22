# Spideybot

## What is this project about?
Spideybot is a quadruped robot designed and simulated as part of a robotics
lab onboarding assignment. The brief was open-ended — build something that
walks — under three hard constraints: no more than eight servos, a 300 mm
cube envelope, and a total mass under 3.5 kg. The project was entirely
digital, meaning nothing was manufactured and all validation was done in
simulation. The final design is a symmetric four-legged robot with two
degrees of freedom per leg, massing 1137 g, whose gait was developed and
measured in MuJoCo.

An earlier concept used six servos with two driven front legs pulling two
passive ball-caster rear legs. That design was abandoned in favour of a
symmetric quadruped, which is more stable, simpler to model, and a better
fit for the creative freedom the brief allowed.

## Hardware
The following components were specified in the design but never assembled,
since the assignment was simulation-only.

- 8 × ST3215-HS bus servos
- Raspberry Pi 5
- Waveshare Bus Servo Adapter A
- INIU 25000 mAh power bank with a USB PD trigger at 12 V for the servo bus

## Kinematics
Each leg has two degrees of freedom. The hip rotates about the Z axis to
sweep the leg through the ground plane, and the knee rotates about the X
axis to lift the foot.

Joint offsets from the chassis origin, in millimetres:

```
hip          (±63.431, ±48.648, +33.70)
hip → knee   (±6.5,    ±50,     −22.5)
knee → foot   75 along y, ±7.5 pad
```

The hip axis is `0 0 1` with a range of −25° to +60°. The knee axis is
`1 0 0` on the front-right and back-right legs and `-1 0 0` on the
front-left and back-left legs, with a range of 30° to 60°. A smaller knee
angle produces a taller stance — 30° gives a 67 mm ride height and 60°
gives 24 mm. All actuators use `kp = 4` and `kv = 0.15`.

## Repository layout
```
spiderbot/
├── venv/
├── assets/          # STL meshes exported from Fusion
├── model.xml        # MJCF, hand-written from measured offsets
├── view.py          # passive viewer
├── demo.py          # gait
├── sweep.py         # knee angle vs ride height
├── check.py         # per-foot height logging
└── Spideybot Assembly.f3z
```

Five meshes were exported from Autodesk Fusion — the chassis, and upper and
lower segments for the left and right legs. Each is scaled by
`scale="0.001 0.001 0.001"` in the MJCF to convert millimetres to metres.
The MJCF itself was written by hand from measured joint offsets rather than
auto-exported, which gave direct control over the joint frames.

## Setup
```
cd ~/Documents/spiderbot
python3.12 -m venv venv
source venv/bin/activate
pip3 install mujoco
```

All paths in `model.xml` and the scripts are relative, so every script must
be run with `spiderbot` as the working directory:

```
mjpython view.py
```

On macOS, `mjpython -m mujoco.viewer` failed under Python 3.14. The scripts
call `mujoco.viewer.launch_passive()` instead, which avoids the GUI
threading issue.

## Gait
`demo.py` pairs same-side legs — front-right with back-right, front-left
with back-left — for forward and backward translation, and switches to crawl
phasing for turns. Measured performance is 235 mm forward and 240 mm
backward, with the ride height holding stable at 71 mm.

Two scripts were written to diagnose the gait. `sweep.py` steps the knee
angle across its range and logs the resulting ride height, which is how the
angle-to-height relationship above was established. `check.py` logs each
foot's height over time, which was used to confirm the legs were phasing as
intended.

## Future work
- Closed-loop foot placement rather than open-loop phase offsets
- Testing on terrain other than a flat plane
- Sim-to-real transfer onto the specified hardware