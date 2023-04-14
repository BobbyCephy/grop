#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import *
from tf.transformations import *
import gurobipy as gp
import os

verbose = True
verboseprint = print if verbose else lambda *args, **kwargs: None


def map_file(name):
    file = os.path.splitext(name)[0] + ".bt"
    path = os.path.join(os.getcwd(), "map", file)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    return path


def mvar_to_var(*args):
    """
    Convert Gurobi MVars to NumPy ndarrays of Vars
    """
    if len(args) == 1:
        try:
            return np.array(args[0].tolist())

        except:
            try:
                if args[0].size == 1:
                    return args[0].item()

                else:
                    return np.fromiter(map(mvar_to_var, args[0]), object)

            except:
                return args[0]

    return map(mvar_to_var, args)


def value(x):
    """Replaces variables in x with solution values"""
    try:
        try:
            return x.getValue()

        except:
            return x.X

    except:
        if len(x) != 1:
            x = np.copy(x)

            for index, item in np.ndenumerate(x):
                try:
                    x[index] = value(item.item())

                except:
                    continue

        return x


def silent_model():
    """Gurobi model without console output"""
    env = gp.Env(empty=True)
    env.setParam("OutputFlag", 0)
    env.start()
    return gp.Model(env=env)


def normalize(vector):
    return vector / np.linalg.norm(vector)


def pose(position=[], orientation=[]):
    return Pose(Point(*position), Quaternion(*orientation))


def eulerToQuaternion(roll=0, pitch=0, yaw=0):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])


def matrixToPose(rotation, position):
    angle = euler_from_matrix(rotation)
    quaternion = quaternion_from_euler(*angle)
    return pose(position, quaternion)


def lookAt(position, target, up=np.array([0, 0, 1]), left=np.array([0, 1e-6, 0])):
    forward = normalize(target - position)
    left = normalize(np.cross(normalize(up), forward) + left)
    up = np.cross(forward, left)
    rotation = np.array([forward, left, up]).T
    return matrixToPose(rotation, position)


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
        namespace[name.split("/")[-1] + suffix("service")] = rospy.ServiceProxy(
            name, service_class
        )


def suffix(x):
    return "_" + str(x) if x else ""


def join_x(x):
    return "x".join(map(str, x))
