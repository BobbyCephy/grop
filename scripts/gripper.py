#!/usr/bin/env python
import numpy as np
import gurobipy as gp


# Halfspace representation of hyperrectangle
def halfspace(bounds):
    try:
        return [halfspace(bound) for bound in bounds]

    except:
        m = len(bounds[0])
        A = []
        b = []

        for i in range(2):
            for j in range(m):
                if bounds[i][j] is not None:
                    a = [0] * m
                    a[j] = (-1) ** (i + 1)
                    A.append(a)
                    b.append((-1) ** (i + 1) * bounds[i][j])

        return np.array(A), np.array(b)


# Big M constant allows each point to lie in grasping space
def dG(gripper, space, d_m, greater=False):
    # Suppress console output
    with gp.Env(empty=True) as env:
        env.setParam("OutputFlag", 0)
        env.start()

        with gp.Model(env=env) as model:
            r = model.addVar(-np.inf)
            q = model.addMVar((gripper.n_q), *gripper.q_lim)

            A_G, b_G = gripper.grasp_space(q, d_m)
            Ab = space(q, d_m)
            D = []

            for A, b in Ab:
                d = []

                for i in range(len(b)):
                    model.setObjective(
                        -r - b[i], gp.GRB.MINIMIZE if greater else gp.GRB.MAXIMIZE
                    )

                    constrs = model.addConstrs(
                        np.dot(A_G[j], A[i]) * r <= b_G[j] for j in range(len(b))
                    )

                    model.optimize()
                    d.append(model.ObjVal)
                    model.remove(constrs)

                D.append(np.array(d))
            return D


class ParallelGripper:
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

    # Half-space representation of bodies
    def bodies(self, q, d_m=0, open=True):
        r_x = self.d_f[0] + d_m + (self.q_lim[1] if open else q[0] + d_m)

        bounds = [
            # Base
            [
                [
                    -self.r_b[0] - d_m,
                    -self.r_b[1] - d_m,
                    -self.r_f[2] - self.d_b[2] - d_m,
                ],
                [self.r_b[0] + d_m, self.r_b[1] + d_m, -self.r_f[2] + d_m],
            ],
            # Finger left
            [
                [-r_x, -self.r_f[1] - d_m, -self.r_f[2] - d_m],
                [-q[0], self.r_f[1] + d_m, self.r_f[2] + d_m],
            ],
            # Finger right
            [
                [q[0], -self.r_f[1] - d_m, -self.r_f[2] - d_m],
                [r_x, self.r_f[1] + d_m, self.r_f[2] + d_m],
            ],
        ]

        Ab = halfspace(bounds)
        self.n_b = len(Ab)
        self.n_bb = [len(ab[1]) for ab in Ab]
        return Ab

    # Spaces near graspings sufaces
    def grasps(self, q, d_m):
        bounds = [
            [
                [-q[0], -self.r_f[1], -self.r_f[2] + d_m],
                [-q[0] + d_m / 2, self.r_f[1], self.r_f[2]],
            ],
            [
                [q[0] - d_m / 2, -self.r_f[1], -self.r_f[2] + d_m],
                [q[0], self.r_f[1], self.r_f[2]],
            ],
        ]

        Ab = halfspace(bounds)
        self.n_g = len(Ab)
        return Ab

    # Space between graspings sufaces
    def grasp_space(self, q, d_m):
        return halfspace(
            [[-q, -self.r_f[1], -self.r_f[2] + d_m], [q, self.r_f[1], self.r_f[2]]]
        )

    # Midpoints of grasping surfaces
    def c_g(self, q):
        return [
            [-q, 0, 0],
            [q, 0, 0],
        ]

    # Maximum number of points in grasping spaces
    def g_max(self, d_m):
        return np.prod([np.floor(self.d_f[i] / d_m + 1) for i in [1, 2]])

    # Maximum radius of grasping space
    def r_g(self, d_m):
        # Suppress console output
        with gp.Env(empty=True) as env:
            env.setParam("OutputFlag", 0)
            env.start()

            with gp.Model(env=env) as model:
                r = model.addMVar((3), -np.inf)
                q = model.addMVar((self.n_q), *self.q_lim)
                model.setObjective(r @ r, gp.GRB.MAXIMIZE)
                A, b = self.grasp_space(q, d_m)
                model.addConstrs(np.array(A[i]) @ r <= b[i] for i in range(len(b)))
                model.optimize()
                r_g = np.sqrt(model.ObjVal)
                return r_g
