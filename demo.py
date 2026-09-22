"""
Main locomotion demo. Runs the robot through four phases — forward, clockwise
turn, backward, anticlockwise turn — using a diagonal trot for translation and
a crawl for turning, and prints displacement, heading change, and body height
after each phase.
"""

import time, math, mujoco, mujoco.viewer

m = mujoco.MjModel.from_xml_path("model.xml")
d = mujoco.MjData(m)

LEGS  = ["FR", "FL", "BR", "BL"]
S_HIP = {"FR": +1, "FL": -1, "BR": +1, "BL": -1}
NEUT  = {"FR": +12, "FL": +12, "BR": -12, "BL": -12}
# trot: diagonal pairs. FR+BL together, FL+BR together.
TROT  = {"FR": 0.0, "BL": 0.0, "FL": 0.5, "BR": 0.5}
# crawl: one leg at a time, used for turning (3 feet down, arcs agree)
CRAWL = {"FL": 0.00, "BR": 0.25, "FR": 0.50, "BL": 0.75}

AMP, TURN_AMP = 20.0, 16.0
KNEE_DOWN, LIFT = 52.0, 12.0
T = 1.2

def hip_id(l):  return m.actuator("a_hip_"  + l).id
def knee_id(l): return m.actuator("a_knee_" + l).id
def smooth(s):  return s * s * (3 - 2 * s)

def profile(p, amp, duty):
    if p < duty:                                   # swing
        s = p / duty
        return -amp + 2 * amp * smooth(s), LIFT * math.sin(math.pi * s)
    s = (p - duty) / (1 - duty)                    # stance
    return amp - 2 * amp * s, 0.0

def drive(t, mode):
    turning = mode in ("cw", "ccw")
    ph   = CRAWL if turning else TROT
    duty = 0.25  if turning else 0.5
    for l in LEGS:
        p = ((t / T) + ph[l]) % 1.0
        if turning:
            sweep, lift = profile(p, TURN_AMP, duty)
            if mode == "cw": sweep = -sweep
            hip = S_HIP[l] * NEUT[l] + sweep
        else:
            sweep, lift = profile(p, AMP, duty)
            if mode == "back": sweep = -sweep
            hip = S_HIP[l] * (NEUT[l] + sweep)
        d.ctrl[hip_id(l)]  = hip
        d.ctrl[knee_id(l)] = KNEE_DOWN - lift

def stand():
    for l in LEGS:
        d.ctrl[hip_id(l)]  = S_HIP[l] * NEUT[l]
        d.ctrl[knee_id(l)] = KNEE_DOWN

def yaw():
    w, x, y, z = d.qpos[3:7]
    return math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

SCRIPT = [("settle", 1.5),
          ("fwd",   7.2), ("settle", 0.8),
          ("cw",    7.2), ("settle", 0.8),
          ("back",  7.2), ("settle", 0.8),
          ("ccw",   7.2), ("settle", 1.5)]

stand()
with mujoco.viewer.launch_passive(m, d) as v:
    for mode, dur in SCRIPT:
        t0 = d.time
        x0, y0, a0 = d.qpos[0], d.qpos[1], yaw()
        while d.time - t0 < dur and v.is_running():
            stand() if mode == "settle" else drive(d.time - t0, mode)
            mujoco.mj_step(m, d)
            v.sync(); time.sleep(0.002)
        da = (yaw() - a0 + 180) % 360 - 180
        print(f"{mode:7s} dx={d.qpos[0]-x0:+.3f} dy={d.qpos[1]-y0:+.3f} "
              f"dyaw={da:+7.1f} z={d.qpos[2]:.3f}")
    while v.is_running():
        stand(); mujoco.mj_step(m, d); v.sync(); time.sleep(0.002)