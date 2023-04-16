#!/usr/bin/env python
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from dimensioning import *
import numpy as np
import gurobipy as gp
from utils import *
from plot import Plot3d
import sage.all as sage
from sage.geometry.polyhedron.plot import cyclic_sort_vertices_2d


class Polytope:
    """Polytope in half-space representation"""

    def __init__(self, A, b, parent=None):
        self.A = np.array(A)
        self.b = np.array(b)
        self.sides, self.dimension = self.A.shape
        self.parent = parent

    def inside(self, p, strict=False):
        return np.all((np.less if strict else np.less_equal)(self.A @ p, self.b))

    def outside(self, p, strict=False):
        return np.any((np.greater if strict else np.greater_equal)(self.A @ p, self.b))

    def constrain_inside(self, p, s, M=None):
        """Constraints for p in Polytope if the s is 1"""
        constr = []

        # Indicator constraints
        if M is None:
            for a, b in zip(self.A, self.b):
                constr += [(s == 1) >> (a @ p <= b)]

        # Big-M constraints
        else:
            for a, b, m in zip(self.A, self.b, M):
                constr += [a @ p <= b + m * (1 - s)]

        return constr

    def constrain_outside(self, p, S, M=None, one=False):
        """Constraints for p out of at least or exactly one side of Polytope"""
        constr = [sum(S) == 1] if one else [sum(S) >= 1]

        # Indicator constraints
        if M is None:
            for a, b, s in zip(self.A, self.b, S):
                constr += [(s == 1) >> (a @ p >= b)]

        # Big-M constraints
        else:
            for a, b, m, s in zip(self.A, self.b, M, S):
                constr += [a @ p >= b + m * (1 - s)]

        return constr

    def get_bounds(self):
        if not hasattr(self, "bounds") or self.bounds is None:
            self.compute_bounds()
        return self.bounds

    def compute_bounds(self):
        model = silent_model()
        P = model.addMVar((2, self.dimension, self.dimension), -np.inf)
        model.setObjective(P[0].diagonal().sum() - P[1].diagonal().sum())
        for p in [*P[0], *P[1]]:
            model.addConstr(self.A @ p <= self.b)
        model.optimize()
        self.bounds = np.array([np.diagonal(p) for p in P.X])
        return self.bounds

    def get_center(self):
        if not hasattr(self, "center") or self.center is None:
            self.compute_center()
        return self.center

    def compute_center(self):
        self.center = np.linalg.lstsq(self.A, self.b)[0]
        return self.center

    def get_diameter(self):
        if not hasattr(self, "diameter") or self.diameter is None:
            self.compute_diameter()
        return self.diameter

    def get_radius(self):
        if not hasattr(self, "radius") or self.radius is None:
            self.radius = self.get_diameter() / 2
        return self.radius

    def compute_diameter(self):
        """Maximum distance between two vertices"""
        model = silent_model()
        vertices = model.addMVar((2, self.dimension), -np.inf)
        model.setObjective(np.square(np.subtract(*vertices)).sum(), gp.GRB.MAXIMIZE)
        model.addConstr(vertices @ self.A.T <= self.b)
        model.Params.NonConvex = 2
        model.optimize()
        self.diameter = np.sqrt(model.ObjVal)
        return self.diameter

    def plot(
        self,
        ax=None,
        T=None,
        text=None,
        vertice_text=None,
        edge_text=None,
        transform=False,
        **kwargs
    ):
        self.get_facets(T, transform)

        if self.facets:
            if not ax:
                plot3d = Plot3d()
                ax = plot3d.ax
                show = True

            else:
                show = False

            polygons = Poly3DCollection(self.facets)
            polygons.set_color("white")
            polygons.set_edgecolor("black")
            polygons.set_alpha(0)
            polygons.set(**kwargs)

            ax.add_collection3d(polygons)

            ax.scatter(*list(zip(*self.vertices)), alpha=0)

            if text:
                ax.text(
                    *np.mean(self.vertices, axis=0).T, text, backgroundcolor="white"
                )

            if vertice_text:
                for vertice in self.vertices:
                    ax.text(*vertice, vertice_text, backgroundcolor="white")

            if edge_text:
                for edge in self.edges:
                    mean = np.mean(edge, axis=0)
                    ax.text(*mean, edge_text, backgroundcolor="white")

            if show:
                plot3d.show()
                plot3d.save()
                return ax

    def get_facets(self, T=None, transform=False):
        A = value(self.A)
        b = value(self.b)

        if T:
            A, b = T.forward(A, b)

        if transform:
            A, b = self.parent.T.inv.transform(A, b)

        ieqs = np.concatenate(([b], -A.T)).T
        P = sage.Polyhedron(ieqs=ieqs)
        self.vertices = P.vertices_list()
        self.facets = [
            [
                list(vertex.vector())
                for vertex in cyclic_sort_vertices_2d(facet.vertices())
            ]
            for facet in P.facets()
        ]


class Rectangle(Polytope):
    """Halfspace representation of hyperrectangle"""

    def __init__(self, *args, **kwargs):
        for key in ["bounds", "min", "max", "size", "center", "vertices"]:
            setattr(self, key, kwargs.get(key, None))

        if self.bounds is not None:
            self.bounds = np.array(self.bounds)
            self.min, self.max = self.bounds
            self.size = self.max - self.min
            self.center = np.mean(self.bounds, axis=0)

        elif self.min is not None and self.max is not None:
            self.bounds = np.array([self.min, self.max])
            self.size = self.max - self.min
            self.center = np.mean(self.bounds, axis=0)

        elif self.size is not None:
            if self.center is not None:
                self.min, self.max = (
                    f(self.center, np.abs(self.size) / 2) for f in [np.subtract, np.add]
                )
                self.bounds = np.array([self.min, self.max])

            else:
                if self.min is not None:
                    self.max = np.add(self.min, self.size)

                elif self.max is not None:
                    self.min = np.subtract(self.max, self.size)

                self.bounds = np.array([self.min, self.max])
                self.center = np.mean(self.bounds, axis=0)

            # Distance of furtest point in cuboid to origin
            self.dmax = np.sqrt(np.sum(np.max(self.bounds**2, axis=0)))
            self.diameter = np.linalg.norm(self.size)
            self.radius = self.diameter / 2

        elif self.vertices is not None:
            self.min, self.max = (f(self.vertices, axis=0) for f in [np.min, np.max])
            self.bounds = np.array([self.min, self.max])
            self.size = self.max - self.min
            self.center = np.mean(self.bounds, axis=0)

        super().__init__(*Rectangle.halfspace(self.bounds), *args)

    def __getitem__(self, index):
        return self.min[0 : len(index)] + self.size[0 : len(index)] * np.clip(
            index, 0, 1
        )

    def from_bounds(Bounds, *args):
        return [Rectangle(bounds=bounds, *args) for bounds in Bounds]

    def halfspace(bounds):
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

        return A, b


if __name__ == "__main__":
    A = [-1, 0, 0], [0, -1, 0], [0, 0, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    b = [1, 2, 3, 4, 5, 6]
    polytope = Polytope(A, b)
    polytope.plot()
