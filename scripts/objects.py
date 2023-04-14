#!/usr/bin/env python
import octomap
import numpy as np
import scipy
import cc3d
from plot import *
from pytransform3d.plot_utils import plot_sphere, plot_box
from polytope import *
from transform import *


def cluster_indices(assignment):
    indices = []

    for i, j in enumerate(assignment):
        while True:
            try:
                indices[j - 1].append(i)
                break
            except:
                indices += [[]]
    return indices


class Objects:
    """Class that manages the occupied voxels as which the objects to be grasped are represented"""

    def __init__(self, space, *args, **kwargs):
        """Initialize with space containing points and map or points"""
        self.space = space

        if args and type(args[0]) == str:
            self.set_map(*args, **kwargs)

        else:
            self.set_points(*args, **kwargs)

    def set_map(self, name="map", limit=None):
        """Obtain occupied voxel coordinates from map file"""
        path = map_file(name)
        objects = octomap.OcTree(1)
        objects.readBinary(bytearray(path, "utf-8"))
        resolution = objects.getResolution()
        points = []

        for leaf in objects.begin_leafs_bbx(*self.space.get_bounds()):
            if limit is not None and len(points) >= limit:
                break

            if objects.isNodeOccupied(leaf):
                point = [leaf.getX(), leaf.getY(), leaf.getZ()]
                # point = [list(leaf.getCoordinate())]

                if type(self.space) is Rectangle or self.space.inside(point):
                    points.append(point)

        self.set_points(points, resolution)

    def set_points(self, points=[], resolution=0.01):
        """Set occupied voxel coordinates"""
        self.points = np.array(points)
        self.n_p = len(self.points)
        if self.n_p:
            print("Number of points:", self.n_p)

        # Diameter of voxel
        self.d_v = resolution
        self.cluster(False)

    def center(self, type=0, tight=False):
        """Points in source frame or centered by substracting their mean"""
        if type and self.n_p:
            if type == 1:
                self.points_mean = self.space.get_center()

            else:
                self.points_mean = np.mean(self.points, axis=0)

            self.points_centered = self.points - self.points_mean
            self.radius = self.space.radius

        else:
            self.points_mean = np.zeros(3)
            self.points_centered = self.points
            self.radius = self.space.dmax

        if tight and self.n_p:
            # Distance of farthest points to origin
            self.radius = np.max(np.linalg.norm(self.points_centered, axis=1))

        self.T = Transform(t=self.points_mean)

        return self.points_centered

    def get_pdist(self):
        if not hasattr(self, "pdist") or self.pdist is None:
            self.pdist = scipy.spatial.distance.pdist(self.points_centered)

        return self.pdist

    def distances(self, tight):
        if self.n_p:
            if tight:
                # Distances between each point and other points
                self.d_pp = scipy.spatial.distance.squareform(self.get_pdist())

                # Greatest distance between each point and other points
                self.d_ppmax = np.max(self.d_pp, axis=0)

                # Greatest distance between two points
                self.d_pmax = np.max(self.d_ppmax)

            else:
                self.d_pmax = self.space.diameter
                self.d_ppmax = np.repeat(self.d_pmax, self.n_p)

    def cluster(self, type=2):
        """Cluster the points into objects"""
        if type == 1:
            self.flat()

        elif type == 2:
            self.connected()

        else:
            self.objects = [list(range(self.n_p))]

        self.n_P = len(self.objects)
        self.n_Pp = [len(object) for object in self.objects]
        self.n_Ppmax = max(self.n_Pp + [0])

        if self.n_p and type:
            print("Points per object:", self.n_Pp)

    def flat(self, s=3):
        linkage = scipy.cluster.hierarchy.linkage(self.get_pdist())
        fcluster = scipy.cluster.hierarchy.fcluster(
            linkage, s * np.sqrt(3) * self.d_v, "distance"
        )
        self.objects = cluster_indices(fcluster)

    def connected(self):
        voxel = np.floor_divide(self.points, self.d_v).astype(int)
        voxel -= voxel.min(axis=0)
        grid = np.full((voxel.max(axis=0) + 1), None)

        for i, v in enumerate(voxel):
            grid[tuple(v)] = i

        labels, self.n_P = cc3d.connected_components(grid != None, return_N=True)
        self.objects = [[] for _ in range(self.n_P)]

        for i, v in enumerate(voxel):
            self.objects[labels[tuple(v)] - 1].append(i)

    def plot_space(self, *args, **kwargs):
        self.space.plot(*args, **kwargs)

    def plot_frame(self, *args, **kwargs):
        self.T.plot(*args, **kwargs, dir=1)

    def plot(self, ax=None, T=None, marker=0):
        if self.n_p:
            show = not ax

            if show:
                ax = Plot.init(self)

            points = self.points

            if T is not None:
                points = T.transform(points)

            for object in self.objects:
                object_points = points[object]

                ax.scatter(*object_points.T)

                if marker == 1:
                    self.cubes(ax, object_points, self.d_v)

                elif marker == 2:
                    self.spheres(ax, object_points, self.d_v)

            if show:
                plt.show()

    def spheres(self, ax, positions, diameter, color="blue"):
        for position in positions:
            plot_sphere(
                ax=ax,
                radius=diameter / 2,
                p=position,
                ax_s=1,
                wireframe=False,
                n_steps=20,
                alpha=1.0,
                color=color,
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
            self.ax.add_collection3d(
                Poly3DCollection(e + position, facecolor=color, alpha=0.5)
            )

    def cubes(self, positions, l, color="blue"):
        plot_box(
            ax=self.ax,
            size=np.full((3), self.diameter),
            A2B=np.eye(4),
            ax_s=1,
            wireframe=True,
            color="k",
            alpha=1.0,
        )


if __name__ == "__main__":
    objects = Objects(source)
    name = "cuboid/0.025x0.1x0.1"
    limit = None
    objects.set_map(name, limit)
    objects.plot()
