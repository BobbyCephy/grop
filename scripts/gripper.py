#!/usr/bin/env python
from abc import ABC, abstractmethod
import numpy as np
import gurobipy as gp
from polytope import *
from plot import *
from grop import *
from utils import *


class Gripper(ABC):
    """Abstract gripper"""

    def __init__(self):
        self.__bodies = None
        self.__grasps = None
        self.__space = None
        self.__c_g = None
        self.__g_max = None

    def plot(
        self,
        ax=None,
        transform=True,
        bodies=2,
        grasps=True,
        centers=True,
        labels=False,
        *args,
        **kwargs
    ):
        show = not ax

        if show:
            ax = Plot3d.new_axes(self)

        if bodies:
            # Representation in opmtimization problem
            if bodies == 1:
                Bodies = zip(self.get_bodies())

            elif bodies > 1:
                # True hull
                bodies_true = self.bodies(self.q, 0, False)

                if bodies == 2:
                    Bodies = zip(bodies_true)

                # Both
                else:
                    Bodies = zip(self.get_bodies(), bodies_true)

            for i, Body in enumerate(Bodies):
                for body in Body:
                    body.plot(
                        ax,
                        text=("b" + str(i)) if labels else "",
                        transform=transform,
                    )

        if grasps:
            for i, grasp in enumerate(self.__grasps):
                grasp.plot(
                    ax,
                    text=("g" + str(i)) if labels else "",
                    transform=transform,
                )

        if centers:
            c_g = value(self.get_c_g())

            if transform:
                c_g = self.T.inv.transform(c_g)

            ax.scatter(*c_g.T, c="0", marker="x")

        if show:
            plt.show()

    @abstractmethod
    def bodies(self):
        """Half-space representation of the convex poleyeders that make up the gripper"""
        ...

    def get_bodies(self, *args, **kwargs):
        if self.__bodies is None:
            self.__bodies = self.bodies(self.q, self.d_v, self.open, *args, **kwargs)
            self.n_b = len(self.__bodies)
            self.n_bb = [polytope.sides for polytope in self.__bodies]
        return self.__bodies

    @abstractmethod
    def grasps(self):
        """Half-space representation of the areas near the grasping surfaces"""
        ...

    def get_grasps(self):
        if self.__grasps is None:
            self.__grasps = self.grasps(self.q)
            self.n_g = len(self.__grasps)
        return self.__grasps

    @abstractmethod
    def space(self):
        """Half-space representation of the grasping surfaces convex hull"""
        ...

    def get_space(self):
        if self.__space is None:
            self.__space = self.space(self.q)
        return self.__space

    @abstractmethod
    def c_g(self):
        """Midpoints of grasping surfaces"""
        ...

    def get_c_g(self):
        if self.__c_g is None:
            self.__c_g = self.c_g(self.q)
        return self.__c_g

    @abstractmethod
    def g_max(self):
        """Maximum number of points in grasping spaces"""
        ...

    def get_g_max(self):
        if self.__g_max is None:
            self.__g_max = self.g_max()
        return self.__g_max

    def r_g(self):
        """Maximum radius of grasping space"""
        model = silent_model()
        r = model.addMVar((3), -np.inf)
        q = model.addMVar((self.n_q), *self.q_lim)
        model.setObjective(r @ r, gp.GRB.MAXIMIZE)
        space = self.space(q)
        model.addConstrs(
            np.array(space.A[i]) @ r <= space.b[i] for i in range(len(space.b))
        )
        model.optimize()
        return np.sqrt(model.ObjVal)

    def dG(self, space, greater=False):
        """Big M constant allows each point to lie in grasping space"""
        model = silent_model()
        r = model.addVar(-np.inf)
        q = model.addMVar((self.n_q), *self.q_lim)

        A_G, b_G = self.space(q)
        Ab = space(q)
        D = []

        for A, b in Ab:
            d = []

            for i in range(len(b)):
                model.setObjective(
                    -r - b[i], gp.GRB.MINIMIZE if greater else gp.GRB.MAXIMIZE
                )

                constrs = model.addConstrs(
                    np.dot(A_G[j], A[i]) * r <= b_G[j] for j in range(len(b_G))
                )

                model.optimize()
                d.append(model.ObjVal)
                model.remove(constrs)

            D.append(np.array(d))
        return D


class ParallelGripper(Gripper):
    def __init__(self, d_f, d_b, d_olim):
        # Diameters of finger [thickness, width, length]
        self.d_f = np.array(d_f)

        # Bounding box diameters of grippers base and the robots last link
        self.d_b = np.array(d_b)

        # Opening width [min, max]
        self.d_olim = np.array(d_olim)

        # Number of actuated joints
        self.n_q = 1

        # Radii
        self.r_f = self.d_f / 2
        self.r_b = self.d_b / 2
        self.q_lim = self.d_olim / 2
        self.r_b = self.d_b / 2

        self.q = self.q_lim[1]
        self.d_v = 0
        self.open = True

        super().__init__()

    def bodies(self, q, d_v, open):
        r_x = self.d_f[0] + d_v + (self.q_lim[1] if open else q[0] + d_v)

        bounds = [
            # Base
            [
                [
                    -self.r_b[0] - d_v,
                    -self.r_b[1] - d_v,
                    -self.r_f[2] - self.d_b[2] - d_v,
                ],
                [
                    self.r_b[0] + d_v,
                    self.r_b[1] + d_v,
                    -self.r_f[2] + d_v,
                ],
            ],
            # Finger left
            [
                [-r_x, -self.r_f[1] - d_v, -self.r_f[2] - d_v],
                [-q[0], self.r_f[1] + d_v, self.r_f[2] + d_v],
            ],
            # Finger right
            [
                [q[0], -self.r_f[1] - d_v, -self.r_f[2] - d_v],
                [r_x, self.r_f[1] + d_v, self.r_f[2] + d_v],
            ],
        ]

        return Rectangle.from_bounds(bounds, self)

    def grasps(self, q):
        bounds = [
            [
                [-q[0], -self.r_f[1], -self.r_f[2] + self.d_v],
                [-q[0] + self.d_v / 2, self.r_f[1], self.r_f[2]],
            ],
            [
                [q[0] - self.d_v / 2, -self.r_f[1], -self.r_f[2] + self.d_v],
                [q[0], self.r_f[1], self.r_f[2]],
            ],
        ]

        return Rectangle.from_bounds(
            bounds,
            self,
        )

    def space(self, q):
        return Rectangle(
            self,
            bounds=[
                [-q, -self.r_f[1], -self.r_f[2] + self.d_v],
                [q, self.r_f[1], self.r_f[2]],
            ],
        )

    def c_g(self, q):
        return [
            [-q, 0, 0],
            [q, 0, 0],
        ]

    def g_max(self):
        return np.prod([np.floor(self.d_f[i] / self.d_v + 1) for i in [1, 2]])
