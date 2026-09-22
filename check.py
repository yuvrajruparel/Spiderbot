"""
Gait phase verification. Runs the trot for 3 seconds and logs each foot's
height every 0.1 s, marking a foot as airborne when it sits clearly above
the lowest one. Used to confirm the legs lift in the intended order rather
than judging it visually.
"""

import math, mujoco

m = mujoco.MjModel.from_xml_path("model.xml")
d = mujoco.MjData(m)

LEGS  = ["FR", "FL", "BR", "BL"]
S_HIP = {"FR": +1, "FL": -1, "BR": +1, "BL": -1}
NEUT  = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
TROT  = {"FR": 0.0, "BL": 0.0, "FL": 0.5, "BR": 0.5}
AMP, KNEE_DOWN, LIFT = 26.0, 52.0, 20.0
DUTY, T = 0.40, 1.4

def hip_id(l):  return m.actuator("a_hip_"  + l).id
def knee_id(l): return m.actuator("a_knee_" + l).id
def foot_z(l):  return d.geom("foot_" + l).xpos[2]
def smooth(s):  return s*s*(3-2*s)

def profile(p):
    if p < DUTY:
        s = p/DUTY
        return -AMP + 2*AMP*smooth(s), LIFT*math.sin(math.pi*s)
    s = (p-DUTY)/(1-DUTY)
    return AMP - 2*AMP*s, 0.0

for l in LEGS:
    d.ctrl[hip_id(l)] = 0
    d.ctrl[knee_id(l)] = KNEE_DOWN
for _ in range(1500):
    mujoco.mj_step(m, d)

print("time   FR     FL     BR     BL    <- foot height, A = airborne")
t0 = d.time
while d.time - t0 < 3.0:
    t = d.time - t0
    for l in LEGS:
        sweep, lift = profile(((t/T) + TROT[l]) % 1.0)
        d.ctrl[hip_id(l)]  = S_HIP[l]*(NEUT[l] + sweep)
        d.ctrl[knee_id(l)] = KNEE_DOWN - lift
    mujoco.mj_step(m, d)
    if int(t*1000) % 100 < 3:
        zs = {l: foot_z(l) for l in LEGS}
        lo = min(zs.values())
        row = "  ".join(f"{zs[l]*1000:5.1f}{'A' if zs[l]-lo > 0.004 else ' '}" for l in LEGS)
        print(f"{t:4.1f}  {row}")