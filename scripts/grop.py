#!/usr/bin/env python
import numpy as np
import gurobipy as gp
from gurobipy import GRB
from polytope import *
from gripper import *
from plot import *
from map import *


def mvar_to_var(*mvars):
    if len(mvars) == 1:
        try:
            if mvars[0].size == 1:
                return mvars[0].item()

            else:
                return np.array([mvar_to_var(mvar) for mvar in mvars[0]])
        except:
            return mvars[0]

    return (mvar_to_var(mvar) for mvar in mvars)


def constraint_rotation(R):
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


def constraint_polytope_in(A, b, M, p, s):
    constr = []

    for k in range(len(b)):
        constr += [A[k] @ p <= b[k] + M[k] * (1 - s)]

    return constr


def constraint_polytope_out(A, b, M, p, s):
    constr = [sum(s) >= 1]

    for k in range(len(b)):
        constr += [A[k] @ p >= b[k] + M[k] * (1 - s[k])]

    return constr


class Grop:
    def __init__(self, gripper, map):
        self.gripper = gripper
        self.map = map
        self.plot = Plot(gripper, map, self)

        # Optimizattion parameters
        self.center = 1
        self.distance = 0
        self.collision = 2
        self.lift = 1
        self.open = 1
        self.pitch = 0
        self.product = 0

    def R_b(self, pitch):
        sin_p, cos_p = (f(np.radians(pitch)) for f in [np.sin, np.cos])
        return np.array(
            [[-1, -1, -sin_p], [-1, -1, -sin_p], [-sin_p, -sin_p, -1]]
        ), np.array([[1, 1, sin_p], [1, 1, sin_p], [sin_p, sin_p, -cos_p]])

    def optimize(self):
        if self.center:
            points = self.map.points_centered

        else:
            points = self.map.points

        # Furtest distance of points to origin
        r_pmax = np.max(np.linalg.norm(points, axis=1))

        self.model = gp.Model("grop")

        # Rotation matrix from world to gripper frame
        self.R_wg = self.model.addMVar((3, 3), *self.R_b(self.pitch))

        # Upper bound of gripper position as furthest point from origin plus maximum radius of grasping space
        t_max = r_pmax + self.gripper.r_g(self.map.d_m)

        # Translation of world to gripper frame
        self.t_wg = self.model.addMVar((3), -t_max, t_max)

        # Opening radius between grasping surfaces
        self.q = self.model.addMVar((self.gripper.n_q), *self.gripper.q_lim)

        # Constrain rotation matrix
        self.model.addConstrs(x for x in constraint_rotation(self.R_wg))

        # Grasping spaces as linear inequalties A @ p <= b
        Ab_g = self.gripper.grasps(self.q, self.map.d_m)

        self.M_g = [
            dG(self.gripper, self.gripper.grasps, self.map.d_m) + np.array(d_ppmax)
            for d_ppmax in self.map.d_ppmax
        ]

        # Bodies as linear inequalties A @ p <= b
        Ab_b = self.gripper.bodies(self.q, self.map.d_m, self.open)

        self.M_b = [
            dG(self.gripper, self.gripper.bodies, self.map.d_m, True)
            - np.array(d_ppmax)
            for d_ppmax in self.map.d_ppmax
        ]

        # Binary variable determines in which grasping space which points lie
        self.s_g = self.model.addMVar(
            (self.gripper.n_g, self.map.n_p), vtype=GRB.BINARY
        )

        # Binary variable determines out of which side of the gripper bodies each point lies
        self.s_b = [
            self.model.addMVar((self.map.n_p, self.gripper.n_bb[i]), vtype=GRB.BINARY)
            for i in range(self.gripper.n_b)
        ]

        ## Number of points per grasping space
        s_gsum = [
            gp.quicksum(self.s_g[i, j] for j in range(self.map.n_p))
            for i in range(self.gripper.n_g)
        ]

        # Maximum number of points per part grasped by a finger
        self.g_max = np.minimum(self.gripper.g_max(self.map.d_m), self.map.n_Ppmax)

        # Maximize number of points per grasping space simultaneously
        if self.product:
            objective_points = s_gsum[0] * s_gsum[1]
            objective_points_max = self.g_max**self.gripper.n_g

        # Maximize sum of number of points grasping space
        else:
            objective_points = gp.quicksum(s_gsum[j] for j in range(self.gripper.n_g))
            objective_points_max = self.g_max * self.gripper.n_g

        # Minimize distances between points and grasping spaces centers
        if self.distance:
            self.d_p = self.model.addMVar((self.gripper.n_g, self.map.n_p))

            objective_distances = self.d_p.sum()

            self.model.setObjectiveN(objective_points, 0, 1, -1)
            self.model.setObjectiveN(objective_distances, 1, 0)

            c_g = self.gripper.c_g(self.q)

        else:
            # Maximize number of points grasped
            self.model.setObjective(objective_points, GRB.MAXIMIZE)

        # Binary variable determines which part is grasped
        self.s_p = self.model.addMVar((self.map.n_P), vtype=GRB.BINARY)

        ## At most one part may be grasped
        self.model.addConstr(sum(self.s_p) <= 1)

        for i in range(self.map.n_P):
            self.model.addConstr(
                sum(self.s_g[:, j] for j in self.map.parts[i])
                <= self.gripper.n_g * len(self.map.parts[i]) * self.s_p[i]
            )

        # Any Points
        if self.map.n_p:
            p = points @ self.R_wg + self.t_wg

            # as variable
            if self.lift or self.collision == 2:
                self.p_ub = (self.map.d_ppmax + self.gripper.r_g(self.map.d_m))[:, None]
                self.p = self.model.addMVar((self.map.n_p, 3), -self.p_ub, self.p_ub)
                self.model.addConstr(self.p == p)

            # as linear expression
            else:
                self.p = p

        else:
            p = []

        # Iterate points
        for i in range(self.map.n_p):
            # Point lies in grasping spaces
            for j in range(self.gripper.n_g):
                for c in constraint_polytope_in(
                    *Ab_g[j], self.M_g[i][j], self.p[i], self.s_g[j][i]
                ):
                    self.model.addConstr(c)

            if self.collision == 1:
                # Point lies outside of bodies
                for j in range(self.gripper.n_b):
                    for c in constraint_polytope_out(
                        *Ab_b[j], self.M_b[i][j], self.p[i], self.s_b[j][i]
                    ):
                        self.model.addConstr(c)

            # Minimize distances between points and grasping spaces centers
            if self.distance:
                for j in range(self.gripper.n_g):
                    self.model.addConstr(
                        self.d_p[i][j]
                        >= sum(
                            (p[k] - c_g[j][k]) * (p[k] - c_g[j][k]) for k in range(3)
                        )
                        - self.map.d_ppmax * (1 - self.s_g[i, j])
                    )

        self.model.Params.NonConvex = 2
        self.model.Params.BestObjStop = objective_points_max
        # self.model.Params.MIPFocus = 1
        # self.model.Params.Heuristics = 0
        # self.model.Params.Presolve = 2
        # self.model.Params.WorkLimit = 10

        def collision_avoidance(model, where):
            if where == GRB.Callback.MIPSOL:
                for name in ["q", "p", "s_b"]:
                    globals()[name] = getattr(model, "_" + name)
                    globals()[name + "X"] = model.cbGetSolution(globals()[name])

                for suffix in ["", "X"]:
                    globals()["Ab_b" + suffix] = self.gripper.bodies(
                        globals()["q" + suffix], self.map.d_m
                    )

                for i in range(len(pX)):
                    for j in range(self.gripper.n_b):
                        # Constraint not added
                        if not any(s_bX[j][i]):
                            # Point inside body
                            if np.all(Ab_bX[j][0] @ pX[i] < Ab_bX[j][1]):
                                for c in constraint_polytope_out(
                                    *mvar_to_var(
                                        *Ab_b[j], self.M_b[i][j], p[i], s_b[j][i]
                                    )
                                ):
                                    model.cbLazy(c)

        if self.collision == 2:
            if self.map.n_p:
                for name in ["q", "p", "s_b"]:
                    setattr(self.model, "_" + name, getattr(self, name))
                self.model.Params.LazyConstraints = 1
                self.model.optimize(collision_avoidance)

        else:
            self.model.optimize()

        return self.solution()

    def solution(self):
        for name in ["R_wg", "t_wg", "q", "s_g", "s_b", "s_p"]:
            var = getattr(self, name)

            if type(var) == list:
                value = [v.X for v in var]

            else:
                value = var.X

            setattr(self, name + ("_" if "_" not in name else "") + "X", value)
            # print("\n", name, ":\n", value)

        # Add points mean to solution
        if self.center and self.map.n_p:
            self.t_wgX -= self.R_wgX @ self.map.points_mean

        self.t_gwX = -self.t_wgX @ self.R_wgX
        self.R_gwX = self.R_wgX.T

        return self.t_gwX, self.R_gwX


if __name__ == "__main__":
    from config import *

    name = "map"
    limit = None
    grop = Grop(gripper, map)
    grop.map.set_map(name, limit=limit)
    grop.optimize()
    grop.plot.points()
    grop.plot.bodies()
    # grop.plot.world_frame()
    # grop.plot.gripper_frame()
    grop.plot.save(name + ("_" + str(limit) if limit else ""))
    grop.plot.show()
