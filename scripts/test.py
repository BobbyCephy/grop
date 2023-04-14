#!/usr/bin/env python
from grop import Grop
from config import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

name = "cuboids"
limit_max = 33

limit_range = list(range(1, limit_max + 1))
collision_range = range(3)

try:
    work = np.loadtxt("test/" + name + ".txt")

except:
    work = np.zeros((1, 3))

for limit in limit_range:
    if work.shape[0] > limit:
        continue

    work = np.append(work, np.zeros((1, 3)), axis=0)

    for collision in collision_range:
        grop = Grop(gripper, objects)
        grop.objects.set_map(name, limit)
        print("limit: ", limit)
        print("collision: ", collision)
        grop.collision = collision
        grop.optimize()
        grop.plot.points()
        grop.plot.bodies()
        grop.plot.save(str(limit), "test/" + str(collision))
        work[limit, collision] = grop.model.Work
        grop.plot.init()

    np.savetxt(name + ".txt", work)

figure = plt.figure()
ax = figure.add_subplot()
index = list(range(0, limit_max + 1))
ax.plot(index, work[index])
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
# ax.set_yscale("log")
ax.set_xlabel("Anzahl von Punkten $n_p$")
ax.set_ylabel("Arbeitseinheiten [s]")
ax.set_title("Nebenbedingungen zur Verhinderung von Kollisionen")
ax.legend(["keine", "alle", "faul"])
figure.savefig("test/" + name + ".svg")
figure.show()
