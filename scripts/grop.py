#!/usr/bin/env python
import numpy as np
import gurobipy as gp
from gurobipy import GRB
from gripper import *
from plot import *
from objects import *
from polytope import *


class RotationMatrixConstraint:
    def exact(R):
        return [
            # First and second rows of rotaton matrix are unit vectors and orthogonal
            R[0] @ R[0] == 1,
            R[1] @ R[1] == 1,
            R[0] @ R[1] == 0,
            # Third row is cross product of first and second
            R[2, 0] == R[0, 1] * R[1, 2] - R[0, 2] * R[1, 1],
            R[2, 1] == R[0, 2] * R[1, 0] - R[0, 0] * R[1, 2],
            R[2, 2] == R[0, 0] * R[1, 1] - R[0, 1] * R[1, 0],
        ]

    def relax(R):
        return []

    def bounds(pitch):
        sin_p, cos_p = (f(np.radians(pitch)) for f in [np.sin, np.cos])
        return np.array(
            [[-1, -1, -sin_p], [-1, -1, -sin_p], [-sin_p, -sin_p, -1]]
        ), np.array([[1, 1, sin_p], [1, 1, sin_p], [sin_p, sin_p, -cos_p]])


class Grop:
    def __init__(self, gripper, objects):
        self.gripper = gripper
        self.objects = objects
        self.gripper.d_v = self.objects.d_v
        self.plot = GripperPlot(gripper, objects, self)

        # Optimizattion parameters
        self.center = 1
        self.distance = 0
        self.collision = 1
        self.lift = 1
        self.open = 1
        self.product = 0
        self.cluster = 1
        self.tight = 0
        self.bigm = 0
        self.one = 0

        self.set_pitch(0)

    def set_pitch(self, pitch):
        self.pitch = pitch
        self.Rb = RotationMatrixConstraint.bounds(pitch)

    def optimize(self):
        """Formulate and solve optimization problem"""
        points = self.objects.center(self.center, self.tight)
        self.objects.distances(self.tight)
        self.objects.cluster(self.cluster)

        self.model = gp.Model("grop")

        self.gripper.open = self.open

        # Opening radius between grasping surfaces
        self.gripper.q = self.model.addMVar((self.gripper.n_q), *self.gripper.q_lim)

        # Rotation matrix from world to gripper frame
        self.R_wg = self.model.addMVar((3, 3), *self.Rb)

        # Constrain rotation matrix
        self.model.addConstrs(c for c in RotationMatrixConstraint.exact(self.R_wg))

        # Upper bound of gripper translation as maximum radius of grasping space plus map radius
        t_max = self.gripper.r_g() + self.objects.radius

        # Translation of world to gripper frame
        self.t_wg = self.model.addMVar((3), -t_max, t_max)

        self.gripper.T = Transform(self.R_wg, self.t_wg)

        # Grasping spaces as linear inequalties A @ p <= b
        self.grasps = self.gripper.get_grasps()

        if self.bigm:
            dGg = self.gripper.dG(self.gripper.grasps)
            self.M_g = [dGg + np.array(x) for x in self.objects.d_ppmax]

        # Bodies as linear inequalties A @ p <= b
        self.bodies = self.gripper.get_bodies()

        if self.bigm or self.collision == 2:
            self.M_b = [
                self.gripper.dG(self.gripper.bodies) - np.array(d_ppmax)
                for d_ppmax in self.objects.d_ppmax
            ]

        # Binary variable determines in which grasping space which points lie
        self.s_g = self.model.addMVar(
            (self.gripper.n_g, self.objects.n_p), vtype=GRB.BINARY
        )

        # Binary variable determines out of which side of the gripper bodies each point lies
        self.s_b = [
            self.model.addMVar(
                (self.objects.n_p, self.gripper.n_bb[i]), vtype=GRB.BINARY
            )
            for i in range(self.gripper.n_b)
        ]

        ## Number of points per grasping space
        s_gsum = [
            gp.quicksum(self.s_g[i, j] for j in range(self.objects.n_p))
            for i in range(self.gripper.n_g)
        ]

        # Maximum number of points per object grasped by a finger
        self.g_max = np.minimum(self.gripper.get_g_max(), self.objects.n_Ppmax)

        # Maximize number of points per grasping space simultaneously
        if self.product:
            objective_points = s_gsum[0] * s_gsum[1]
            objective_points_max = self.g_max**self.gripper.n_g

        # Maximize sum of number of points grasping space
        else:
            objective_points = gp.quicksum(s_gsum[j] for j in range(self.gripper.n_g))
            objective_points_max = self.g_max * self.gripper.n_g

        # Maximize number of points grasped
        self.model.ModelSense = GRB.MAXIMIZE
        self.model.setObjectiveN(objective_points, 0, 1)

        # Minimize distances between points and grasping surfaces centers
        if self.distance:
            self.d_p = self.model.addMVar((self.gripper.n_g, self.objects.n_p))
            objective_distances = self.d_p.sum()
            self.model.setObjectiveN(objective_distances, 1, 0, -1)
            c_g = self.gripper.get_c_g()

            if not self.bigm:
                self.d_pa = self.model.addMVar(self.d_p.shape)

        # At most one object may be grasped
        if self.cluster:
            # Binary variable determines which object is grasped
            self.s_p = self.model.addMVar((self.objects.n_P), vtype=GRB.BINARY)
            self.model.addConstr(gp.quicksum(self.s_p) <= 1)

            for i in range(self.objects.n_P):
                self.model.addConstr(
                    gp.quicksum(self.s_g[:, j] for j in self.objects.objects[i])
                    <= self.gripper.n_g * len(self.objects.objects[i]) * self.s_p[i]
                )

        # Points in gripper frame
        if self.objects.n_p:
            self.points = points @ self.R_wg + self.t_wg

            # as variable
            if self.lift or self.collision == 2:
                self.p = self.model.addMVar((self.objects.n_p, 3), -np.Inf)
                self.model.addConstr(self.p == self.points)

            # as expression
            else:
                self.p = self.points

        else:
            self.p = []

        # Iterate points
        for i in range(self.objects.n_p):
            # Point lies in grasping spaces
            for j in range(self.gripper.n_g):
                self.model.addConstrs(
                    c
                    for c in self.grasps[j].constrain_inside(
                        self.p[i], self.s_g[j][i], self.M_g[i][j] if self.bigm else None
                    )
                )

            if self.collision == 1:
                # Point lies outside of bodies
                for j in range(self.gripper.n_b):
                    for c in self.bodies[j].constrain_outside(
                        self.p[i],
                        self.s_b[j][i],
                        self.M_b[i][j] if self.bigm else None,
                        one=self.one,
                    ):
                        self.model.addConstr(c)

            # Minimize distances between points and grasping spaces centers
            if self.distance:
                for j in range(self.gripper.n_g):
                    if self.bigm:
                        self.model.addConstr(
                            self.d_p[j][i]
                            >= gp.quicksum(
                                (self.p[i][k] - c_g[j][k]) * (self.p[i][k] - c_g[j][k])
                                for k in range(3)
                            )
                            - self.objects.d_ppmax * (1 - self.s_g[j, i])
                        )

                    else:
                        self.model.addConstr(
                            self.d_pa[j][i]
                            >= gp.quicksum(
                                (self.p[i][k] - c_g[j][k]) * (self.p[i][k] - c_g[j][k])
                                for k in range(3)
                            )
                        )

                        self.model.addConstr(
                            (self.s_g[j, i] == 1) >> (self.d_p[j][i] == self.d_pa[j][i])
                        )

        self.model.Params.NonConvex = 2
        self.model.Params.BestObjStop = objective_points_max
        # self.model.Params.MIPFocus = 1
        # self.model.Params.Heuristics = 0
        # self.model.Params.Presolve = 2
        # self.model.Params.WorkLimit = 1

        def collision_avoidance(model, where):
            if where == GRB.Callback.MIPSOL:
                for name in ["q", "p", "s_b"]:
                    globals()[name] = getattr(model, "_" + name)
                    globals()[name + "X"] = model.cbGetSolution(globals()[name])
                    globals()[name] = mvar_to_var(globals()[name])

                for suffix in ["", "X"]:
                    globals()["bodies" + suffix] = self.gripper.bodies(
                        globals()["q" + suffix], self.objects.d_v
                    )

                for i in range(len(pX)):
                    for j in range(self.gripper.n_b):
                        # Constraint not added
                        if not any(s_bX[j][i]):
                            # Point strictly inside body
                            if bodiesX[j].inside(pX[i], True):
                                for c in bodies[j].constrain_outside(
                                    p[i], s_b[j][i], self.M_b[i][j]
                                ):
                                    model.cbLazy(c)

        if self.collision == 2:
            if self.objects.n_p:
                for name in ["q", "p", "s_b"]:
                    setattr(self.model, "_" + name, getattr(self, name))

                self.model.Params.LazyConstraints = 1
                self.model.optimize(collision_avoidance)

        else:
            self.model.optimize()

        return self.solution()

    def solution(self):
        for name in ["R_wg", "t_wg", "q", "s_g", "s_b", "s_p"]:
            try:
                var = getattr(self, name)

                if type(var) == list:
                    value = [v.X for v in var]

                else:
                    value = var.X

                setattr(self, name + ("_" if "_" not in name else "") + "X", value)
                # print("\n", name, ":\n", value)

            except:
                continue

        # Add points mean to solution
        if self.center and self.objects.n_p:
            self.t_wgX -= self.R_wgX @ self.objects.points_mean

        self.gripper.T = Transform(self.R_wgX, self.t_wgX)

        return self.gripper.T.inv


if __name__ == "__main__":
    from config import *

    name = "map"
    limit = 10
    grop = Grop(gripper, objects)
    grop.objects.set_map(name, limit)
    grop.optimize()
    grop.plot.in_gripper_frame = False
    grop.plot.points()
    grop.plot.bodies()
    # grop.plot.world_frame()
    # grop.plot.gripper_frame()
    # grop.plot.source()
    # grop.plot.source_frame()
    grop.plot.save(name + suffix(limit))
    grop.plot.show()
