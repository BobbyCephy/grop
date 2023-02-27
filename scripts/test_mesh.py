#!/usr/bin/env python
from grop import Grop
from config import *

grop = Grop(gripper, map)

for name in ["Lego_brick"]:
    limit = 40
    grop.map.set_map(name, "mesh", limit=limit)
    grop.optimize()
    grop.plot.points()
    grop.plot.bodies()
    grop.plot.save(name + str(limit), "mesh")
    grop.plot.show()
