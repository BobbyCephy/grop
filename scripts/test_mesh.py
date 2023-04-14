#!/usr/bin/env python
from grop import Grop
from config import *

grop = Grop(gripper, objects)

for name in ["Lego_brick"]:
    limit = 40
    grop.objects.set_map("mesh" + "/" + name, limit)
    grop.optimize()
    grop.plot.points()
    grop.plot.bodies()
    grop.plot.save(name + str(limit), "mesh")
    grop.plot.show()
