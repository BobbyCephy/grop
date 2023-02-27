#!/usr/bin/env python
from grop import Grop
from config import *

d_olim[0] = d_olim[1]  # = np.mean(d_olim)
gripper = ParallelGripper(d_f, d_b, d_olim)

grop = Grop(gripper, map)
grop.map.set_points([[0.1] * 3])
grop.optimize()

grop.plot.in_gripper_frame = False
grop.plot.axes.scatter(*grop.t_wgX, alpha=0)
grop.plot.bodies()
grop.plot.world_frame()
grop.plot.gripper_frame()

grop.plot.axes.view_init(15, -60)
grop.plot.save("frame")
grop.plot.show()
