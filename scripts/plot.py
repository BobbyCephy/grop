#!/usr/bin/env python
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pytransform3d.rotations import plot_basis
from polytope import *
from dimensioning import *
from gripper import *


class Plot:
    def __init__(self, gripper, map, grop):
        self.gripper = gripper
        self.map = map
        self.grop = grop
        self.in_gripper_frame = False
        self.init()

    def init(self):
        self.figure = plt.figure()
        self.axes = self.figure.add_subplot(projection="3d")
        self.axis = [getattr(self.axes, a + "axis") for a in ["x", "y", "z"]]
        self.axes.axis("off")
        return

        for a in ["x", "y", "z"]:
            getattr(self.axes, "set_" + a + "label")(a)

        for axis in self.axis:
            axis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    def set_limits(self, limits):
        for i, axis, _ in enumerate(self.axis):
            axis.set_lim(limits[:, i])

    def no_ticks(self):
        for axis in self.axis:
            axis.set_ticks([], [])

    def pose(self, lengthAxes=0.1):
        color = ["red", "green", "blue"]

        for i in range(3):
            x, y, z = zip(
                self.grop.t_gwX, self.grop.t_gwX + lengthAxes * self.grop.R_gwX[i]
            )
            self.axes.plot(x, y, z, color[i], linewidth=3)

    def gripper_frame(self, s=0.1):
        if self.in_gripper_frame:
            plot_basis(self.axes, s=s)
        else:
            plot_basis(self.axes, self.grop.R_gwX, self.grop.t_gwX, s=s)

    def world_frame(self, s=0.1):
        if self.in_gripper_frame:
            plot_basis(self.axes, self.grop.R_wgX, self.grop.t_wgX, s=s)
        else:
            plot_basis(self.axes, s=s)

    def spheres(self, positions, diameter, color="blue", resolution=10j):
        radius = diameter / 2
        u, v = np.mgrid[0 : 2 * np.pi : 2 * resolution, 0 : np.pi : resolution]
        x = radius * np.cos(u) * np.sin(v)
        y = radius * np.sin(u) * np.sin(v)
        z = radius * np.cos(v)

        for position in positions:
            self.axes.plot_surface(
                x + position[0], y + position[1], z + position[2], color=color, alpha=1
            )

    def cubes(self, positions, l, color="blue"):
        e = (
            np.array(
                [
                    [[0, l, 0], [0, 0, 0], [l, 0, 0], [l, l, 0]],
                    [[0, 0, 0], [0, 0, l], [l, 0, l], [l, 0, 0]],
                    [[l, 0, l], [l, 0, 0], [l, l, 0], [l, l, l]],
                    [[0, 0, l], [0, 0, 0], [0, l, 0], [0, l, l]],
                    [[0, l, 0], [0, l, l], [l, l, l], [l, l, 0]],
                    [[0, l, l], [0, 0, l], [l, 0, l], [l, l, l]],
                ]
            )
            - l / 2
        )

        for position in positions:
            self.axes.add_collection3d(
                Poly3DCollection(e + position, facecolor=color, alpha=0.5)
            )

    def source(self):
        self.cuboids(self.source.size, [self.source.center], alpha=0.05)

    def points(self, marker=0):
        if self.map.n_p:
            if self.in_gripper_frame:
                points = self.map.points @ self.grop.R_wgX + self.grop.t_wgX

            else:
                points = self.map.points

            for part in self.map.parts:
                part_points = points[part]

                self.axes.scatter(
                    part_points[:, 0], part_points[:, 1], part_points[:, 2]
                )

                if marker == 1:
                    self.cubes(part_points, self.map.d_m)

                elif marker == 2:
                    self.spheres(part_points, self.map.d_m)

    def scale(self):
        self.axes.relim()
        self.axes.autoscale()
        self.axes.set_aspect("equal", "box")
        plt.tight_layout()

    def save(self, name="figure", folder="", limit=None):
        self.scale()
        path = os.path.join(os.getcwd(), "figure", folder)
        os.makedirs(path, exist_ok=True)
        plt.savefig(os.path.join(path, name + ".svg"))

    def show(self):
        self.scale()
        plt.show()

    def cuboid(self, A, b, **kwargs):
        if not self.in_gripper_frame:
            A = A @ self.grop.R_gwX
            b = b + A @ self.grop.t_gwX

        Polytope(A, b).plot(self.axes, **kwargs)

    def bodies(self):
        bodies = self.gripper.bodies(self.grop.q_X, open=False)

        for i in range(len(bodies)):
            self.cuboid(*bodies[i], label="Bodies" + str(i))
