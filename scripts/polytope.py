import cdd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from dimensioning import *
import numpy as np


class Polytope:
    def __init__(self, A, b):
        matrix = cdd.Matrix([[b[i], *(-a for a in A[i])] for i in range(len(b))])
        matrix.rep_type = cdd.RepType.INEQUALITY
        self.polyhedron = cdd.Polyhedron(matrix)
        self.vertices = [list(x[1:]) for x in self.polyhedron.get_generators() if x[0]]
        self.adjacencies = [list(x) for x in self.polyhedron.get_adjacency()]
        self.incidences = [list(x) for x in self.polyhedron.get_input_incidence() if x]
        self.sort_incidences()
        self.faces = [
            [self.vertices[index] for index in incidence]
            for incidence in self.incidences
        ]
        self.get_edges()

    def get_edges(self):
        self.edges_indices = []

        for i in range(len(self.adjacencies)):
            for j in self.adjacencies[i]:
                edge = {i, j}

                if edge not in self.edges_indices:
                    self.edges_indices.append(edge)

        self.edges = [
            [self.vertices[index] for index in indice] for indice in self.edges_indices
        ]

    def sort_incidences(self):
        for incidence in self.incidences:
            incidenceSorted = [incidence.pop(0)]

            while incidence:
                for index in incidence:
                    if index in self.adjacencies[incidenceSorted[-1]]:
                        incidence.remove(index)
                        incidenceSorted.append(index)
                        break

            incidence += incidenceSorted
        return self.incidences

    def plot(self, axes=None, vertice_text=None, edge_text=None, **kwargs):
        if self.vertices:
            if not axes:
                figure = plt.figure()
                axes = figure.add_subplot(projection="3d")
                axes.set_xlabel("x")
                axes.set_ylabel("y")
                axes.set_zlabel("z")
                show = True

            else:
                show = False

            polygons = Poly3DCollection(self.faces)
            polygons.set_color("white")
            polygons.set_edgecolor("black")
            polygons.set_alpha(0)
            polygons.set(**kwargs)

            axes.add_collection3d(polygons)

            axes.scatter(*list(zip(*self.vertices)), alpha=0)

            if vertice_text:
                for vertice in self.vertices:
                    axes.text(*vertice, vertice_text, backgroundcolor="white")

            if edge_text:
                for edge in self.edges:
                    mean = np.mean(edge, axis=0)
                    axes.text(*mean, edge_text, backgroundcolor="white")

            if show:
                axes.set_aspect("equal", "box")
                plt.show()
                return axes


if __name__ == "__main__":
    A = [-1, 0, 0], [0, -1, 0], [0, 0, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    b = [1, 2, 3, 4, 5, 6]
    polytope = Polytope(A, b)
    polytope.plot()
