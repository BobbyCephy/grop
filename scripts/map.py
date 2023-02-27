#!/usr/bin/env python
import octomap
import numpy as np
import scipy.spatial
import os


class Map:
    def __init__(self, space, *args, **kwargs):
        # Cuboid space containing points to grip
        self.space = space

        # Diameter of space containing parts to pick
        self.d_s = np.linalg.norm(space.size)

        if args and type(args[0]) == str:
            self.set_map(*args, **kwargs)

        else:
            self.set_points(*args, **kwargs)

    def set_map(self, name="map", folder="", limit=None):
        path = os.path.join(os.getcwd(), "map", folder)
        file = os.path.join(path, name + ".bt")
        map = octomap.OcTree(1)
        map.readBinary(bytearray(file, "utf-8"))
        resolution = map.getResolution()
        points = []

        for leaf in map.begin_leafs_bbx(*self.space.limits):
            if limit is not None and len(points) >= limit:
                break

            if map.isNodeOccupied(leaf):
                points.append([leaf.getX(), leaf.getY(), leaf.getZ()])

        self.set_points(points, resolution)

    def set_points(self, points=[], resolution=0.01):
        # Number of points
        self.n_p = len(points)
        print("Number of points:", self.n_p)

        # Diameter of map point
        self.d_m = resolution

        # Radius of map point
        self.r_m = self.d_m / 2

        if points:
            self.points = np.array(points)
        else:
            self.points = None

        self.center()
        self.cluster()
        self.statistics()

    def center(self):
        if self.points is None:
            self.points_mean = np.array([0] * 3)
            self.points_centered = None
        else:
            # Subtract points mean for lowest M values
            self.points_mean = np.mean(self.points, axis=0)
            self.points_centered = self.points - self.points_mean

    def cluster(self):
        if self.points is not None:
            # Distances between each point and other points
            self.d_pp = scipy.spatial.distance_matrix(self.points, self.points)

        # Point indices of parts
        self.parts = []

        # Point indices
        point_indices = list(range(self.n_p))

        while point_indices:
            self.parts.append([point_indices.pop(0)])

            for i in self.parts[-1]:
                for j in point_indices:
                    if (self.d_pp[i][[j]]) <= np.sqrt(3) * self.d_m:
                        point_indices.remove(j)
                        self.parts[-1].append(j)

        self.n_P = len(self.parts)
        self.n_Pp = [len(part) for part in self.parts]
        self.n_Ppmax = max(self.n_Pp + [0])

        print("Points per part:", self.n_Pp)

    def statistics(self):
        if self.points is not None:
            # Greatest distance between point and other point
            self.d_ppmax = np.max(self.d_pp, axis=0)

            # Greatest distance between two points
            self.d_pmax = np.max(self.d_ppmax)

        else:
            self.d_ppmax = np.inf
            self.d_pmax = np.inf
