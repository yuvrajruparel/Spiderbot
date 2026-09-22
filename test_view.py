"""
Environment check. Drops a box onto a plane to confirm MuJoCo and the viewer
are working before loading the robot model.
"""

import time, mujoco, mujoco.viewer
m = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><light pos="0 0 3"/>'
    '<geom type="plane" size="2 2 .1"/>'
    '<body pos="0 0 .5"><freejoint/>'
    '<geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>')
d = mujoco.MjData(m)
with mujoco.viewer.launch_passive(m, d) as v:
    while v.is_running():
        mujoco.mj_step(m, d)
        v.sync()
        time.sleep(0.002)
