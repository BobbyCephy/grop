#!/usr/bin/env python
from config import *
from grop import Grop

if __name__ == "__main__":
    from config import *

    name = "cuboids"
    map.set_map(name)
    grop = Grop(gripper, map)
    grop.plot.points()
    grop.plot.world_frame()
    grop.plot.save("cluster")
    grop.plot.show()
