#!/usr/bin/env python
from dimensioning import *
import numpy as np
from pytransform3d.rotations import plot_basis
from utils import *
from plot import *


class Transform:
    """Rotation and translation"""

    def __init__(self, R=np.eye(3), t=np.zeros(3), inv=None):
        self.R, self.t = R, t
        self.inv = Transform(R.T, -t @ R, self) if inv is None else inv

    def __iter__(self):
        return self.R, self.t

    def transform(self, *args):
        if len(args) == 1:
            return self.transform_point(*args)

        elif len(args) == 2:
            return self.transform_halfspace(*args)

    def transform_point(self, p):
        return p @ self.R + self.t

    def transform_halfspace(self, A, b):
        A = A @ self.R
        b = b + A @ self.t
        return A, b

    def plot(self, ax, T=None, dir=0, s=0.1):
        if dir < 0:
            Rt = self.inv
        elif dir > 0:
            Rt = self
        else:
            Rt = ()
        if T:
            Rt = self.transform(*T)
        plot_basis(ax, *Rt, s=s)

    def pose(self):
        return matrixToPose(*self.__iter__())
