#!/usr/bin/env python
from grop import Grop
from config import *

d_olim[0] = d_olim[1]  # = np.mean(d_olim)
gripper = ParallelGripper(d_f, d_b, d_olim)

grop = Grop(gripper, objects)
grop.objects.set_points([[0] * 3] * 3)
grop.optimize()

grop.plot.in_gripper_frame = True
grop.plot.ax.set_proj_type("ortho")
grop.plot.ax.scatter(*grop.t_wgX, alpha=0)
gripper.plot(ax=grop.plot.ax, transform=not grop.plot.in_gripper_frame, bodies=3)
grop.plot.gripper_frame()
grop.plot.save("gripper/gripper")
grop.plot.show()
