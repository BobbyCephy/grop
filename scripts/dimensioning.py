#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np


def dimensioning(ax, p, text=None):
    arrowprops = dict(arrowstyle="<->", shrinkA=0, shrinkB=0)
    ax.annotate("", *p, arrowprops=arrowprops)

    diff = np.diff(p, axis=0)[0]
    mean = np.mean(p, axis=0)
    rotation = np.rad2deg(np.arctan2(diff[1], diff[0]))

    if text is None:
        text = str(np.linalg.norm(diff))

    ax.text(*mean, text, rotation=rotation)


if __name__ == "__main__":

    def f(x):
        return x, np.sin(x)

    x = np.linspace(0, 2 * np.pi, 100)
    plt.plot(*f(x))
    ax = plt.gca()
    ax.set_aspect("equal")
    dimensioning(ax, [f(0), f(3 / 2 * np.pi)])
    plt.show()
