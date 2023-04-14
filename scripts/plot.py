#!/usr/bin/env python
import os
import matplotlib.pyplot as plt
from utils import *


class Plot3d:
    def __init__(self):
        self.new_axes()

    def new_axes(self, blank=True):
        self.fig = plt.figure(layout="constrained")
        self.ax = self.fig.add_subplot(projection="3d")

        if blank:
            self.set_blank()

        else:
            self.axis = [getattr(self.ax, a + "axis") for a in ["x", "y", "z"]]
            self.set_label()
            self.set_color()
            self.no_ticks()

        self.set_view()

        return self.ax

    def set_blank(self):
        self.ax.axis("off")

    def set_label(self):
        for axis in self.axis:
            axis.set_label_text(axis.adir)

    def set_color(self):
        for axis in self.axis:
            axis.set_pane_color([0] * 4)

    def no_ticks(self):
        for axis in self.axis:
            axis.set_ticks([], [])

    def set_limits(self, limits):
        for i, axis in enumerate(self.axis):
            axis.set_lim(limits[:, i])

    def scale(self):
        # self.ax.margins(0)
        self.ax.set_aspect("equal", "box")

    views = {
        "": (),
        "xy": (90, -90),
        "xz": (0, -90),
        "yz": (0, 0),
        "-xy": (-90, 90),
        "-xz": (0, 90),
        "-yz": (90, 180),
        "180": (30, 120),
    }

    def set_view(self, *args, **kwargs):
        if args and type(args[0]) == str:
            self.view = args[0]
            args = Plot3d.views.get(self.view, ())

        elif args or kwargs:
            self.view = "_".join([*map(str, self.get_view())])

        else:
            self.view = ""

        self.ax.view_init(*args, **kwargs)

    def get_view(self):
        return self.ax.elev, self.ax.azim, self.ax.roll

    def save(
        self,
        name="figure",
        views=None,
        format="svg",
        bbox_inches="tight",
        pad_inches=0,
        transparent=True,
        **kwargs
    ):
        base = os.path.join(os.getcwd(), "figure", name)
        os.makedirs(os.path.dirname(base), exist_ok=True)

        if not views:
            views = Plot3d.views

        for view in views:
            self.set_view(view)
            file = base + suffix(self.view) + "." + format
            self.scale()

            plt.savefig(
                file,
                format=format,
                bbox_inches=bbox_inches,
                pad_inches=pad_inches,
                transparent=transparent,
                **kwargs,
            )

    def show(self):
        self.scale()
        plt.show()


class GripperPlot(Plot3d):
    def __init__(self, gripper=None, objects=None, grop=None):
        self.gripper = gripper
        self.objects = objects
        self.grop = grop
        self.in_gripper_frame = False
        super().__init__()

    def gripper_frame(self):
        if self.in_gripper_frame:
            self.gripper.T.plot(self.ax)
        else:
            self.gripper.T.plot(self.ax, dir=-1)

    def world_frame(self):
        if self.in_gripper_frame:
            self.gripper.T.plot(self.ax, dir=1)
        else:
            Transform.plot(self.ax)

    def bodies(self):
        self.gripper.plot(self.ax, not self.in_gripper_frame)

    def points(self):
        self.objects.plot(self.ax, self.gripper.T if self.in_gripper_frame else None)

    def source(self):
        self.objects.plot_space(
            self.ax, self.gripper.T if self.in_gripper_frame else None
        )

    def source_frame(self):
        self.objects.plot_frame(
            self.ax, self.gripper.T if self.in_gripper_frame else None
        )
