#!/usr/bin/env python
from config import *
from grop import Grop

if __name__ == "__main__":
    from config import *

    name = "cuboids"
    objects.set_map(name)
    objects.cluster()
    grop = Grop(gripper, objects)
    grop.plot.points()
    #grop.plot.world_frame()
    grop.plot.save("cluster")
    grop.plot.show()
