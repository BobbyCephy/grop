#!/usr/bin/env python
import numpy as np
from utils import *
from gripper import *
from objects import *
from polytope import *

desk_size = np.array([1.08, 1.23, 1])
desk_min = -np.array([0.255, 0.365, desk_size[2]])

desk = Rectangle(size=desk_size, min=desk_min)

floor_size = np.array([10, 10, 0.01])
floor_min = -np.array([1, 1, desk_size[2] + floor_size[2]])

floor = Rectangle(size=floor_size, min=floor_min)

space_size = np.copy(desk_size)
space_size[2] = 0.4
space_min = np.copy(desk_min)
space_min[2] = 0

space = Rectangle(size=space_size, min=space_min)

diameter = space.compute_diameter()

quarter_size = space_size * [1 / 2, 1 / 2, 1]
quarter_centers = [
    space[(2 * x + 1) / 4, (2 * y + 1) / 4, 1 / 2] for x, y in [(1, 0), (1, 1), (0, 1)]
]

quarter = [Rectangle(size=quarter_size, center=center) for center in quarter_centers]

source = quarter[0]
destination = quarter[1]

gripper_name = "robotiq_hande"

if gripper_name == "robotiq85":
    d_f = np.array([0.0065, 0.027, 0.038])
    d_b = np.array([0.1527, 0.075, 0.1628])
    d_olim = [0, 0.085]

elif gripper_name == "robotiq_hande":
    d_f = np.array([0.006, 0.021, 0.0455])
    d_b = np.array([0.075, 0.075, 0.146])
    d_olim = [0, 0.05]

gripper = ParallelGripper(d_f, d_b, d_olim)
objects = Objects(source)
