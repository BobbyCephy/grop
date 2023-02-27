#!/usr/bin/env python
from grop import Grop
from config import *

d_olim[0] = d_olim[1]  # = np.mean(d_olim)
gripper = ParallelGripper(d_f, d_b, d_olim)

grop = Grop(gripper, map)
grop.map.set_points([[0] * 3])
grop.optimize()

grop.plot.in_gripper_frame = True
grop.plot.axes.set_proj_type("ortho")
grop.plot.axes.scatter(*grop.t_wgX, alpha=0)
grop.plot.bodies()
grop.plot.gripper_frame()

for azim, plane in [(90, "xz"), (0, "yz")]:
    grop.plot.axes.view_init(0, azim)
    grop.plot.save(plane, "gripper")

grop.plot.show()
