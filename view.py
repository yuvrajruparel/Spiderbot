import time, math, mujoco, mujoco.viewer

m = mujoco.MjModel.from_xml_path("model.xml")
d = mujoco.MjData(m)

for leg in ["FR", "FL", "BR", "BL"]:
    d.qpos[m.joint("hip_"  + leg).qposadr.item(0)] = math.radians(0)
    d.qpos[m.joint("knee_" + leg).qposadr.item(0)] = math.radians(45)
mujoco.mj_forward(m, d)

with mujoco.viewer.launch_passive(m, d) as v:
    while v.is_running():
        mujoco.mj_forward(m, d)   # no stepping — static pose check
        v.sync()
        time.sleep(0.01)