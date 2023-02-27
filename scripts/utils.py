#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import *
from tf.transformations import *


class Cuboid:
    def __init__(self, **kwargs):
        for key in ["size", "center", "min", "max", "corners"]:
            setattr(self, key, kwargs.get(key, []))

        if len(self.size):
            if len(self.center):
                self.min, self.max = (
                    f(self.center, np.abs(self.size) / 2) for f in [np.subtract, np.add]
                )

            else:
                if len(self.min):
                    self.max = np.add(self.min, self.size)

                elif len(self.max):
                    self.min = np.subtract(self.max, self.size)

                self.center = np.mean([self.min, self.max], axis=0)

        elif len(self.corners):
            self.min, self.max = (f(self.corners, axis=0) for f in [np.min, np.max])
            self.size = self.max - self.min

        self.limits = np.array([self.min, self.max])

    def __getitem__(self, index):
        return self.min[0 : len(index)] + self.size[0 : len(index)] * np.clip(
            index, 0, 1
        )


def normalize(vector):
    return vector / np.linalg.norm(vector)


def pose(position=[], orientation=[]):
    return Pose(Point(*position), Quaternion(*orientation))


def eulerToQuaternion(roll=0, pitch=0, yaw=0):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])


def matrixToPose(position, rotation):
    angle = euler_from_matrix(rotation)
    quaternion = quaternion_from_euler(*angle)
    return pose(position, quaternion)


def lookAt(position, target, up=np.array([0, 0, 1]), left=np.array([0, 1e-6, 0])):
    forward = normalize(target - position)
    left = normalize(np.cross(normalize(up), forward) + left)
    up = np.cross(forward, left)
    rotation = np.array([forward, left, up]).T
    return matrixToPose(position, rotation)


def lookAtPath(positions, targets, numberPoints=10, relative=False, back=True):
    numberPositions = len(positions)
    path = []

    for indexPosition in range(numberPositions - (not back)):
        for indexPoint in range(numberPoints):
            fraction = indexPoint / numberPoints
            position = (1 - fraction) * positions[indexPosition] + fraction * positions[
                (indexPosition + 1) % numberPositions
            ]
            if np.shape(targets) == np.shape(positions):
                target = (1 - fraction) * targets[indexPosition] + fraction * targets[
                    (indexPosition + 1) % numberPositions
                ]
            else:
                target = np.copy(targets)
            if relative:
                target += position
            path.append(lookAt(position, target))

    return path


def service(namespace, *names_classes, wait=True):
    for name, service_class in names_classes:
        if wait:
            rospy.wait_for_service(name, 3)
        namespace[name.split("/")[-1]] = rospy.ServiceProxy(name, service_class)
