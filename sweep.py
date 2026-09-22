"""
Standing height sweep. Settles the robot at knee angles from 30 to 60 degrees
and reports the resulting body height and foot positions, used to choose a
stance angle that balances ride height against how far the legs splay out.
"""

import mujoco

m = mujoco.MjModel.from_xml_path("model.xml")
LEGS = ["FR", "FL", "BR", "BL"]

print("knee   body_z   FR_z   FL_z   BR_z   BL_z   (metres)")
for ang in range(30, 61, 3):
    d = mujoco.MjData(m)
    for l in LEGS:
        d.ctrl[m.actuator("a_hip_"  + l).id] = 0.0
        d.ctrl[m.actuator("a_knee_" + l).id] = ang
    for _ in range(4000):
        mujoco.mj_step(m, d)
    feet = "  ".join(f"{d.geom('foot_'+l).xpos[2]:.3f}" for l in LEGS)
    print(f"{ang:4d}   {d.qpos[2]:.3f}   {feet}")