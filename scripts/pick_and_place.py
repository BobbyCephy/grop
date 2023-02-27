#!/usr/bin/env python
from sqlite3 import OperationalError
import sys
import os
import copy
from tkinter import Y
import rospy
import moveit_commander
import moveit_msgs.srv
from std_msgs.msg import *
from geometry_msgs.msg import *
from moveit_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.srv import *
from std_srvs.srv import *
import numpy as np
import matplotlib.pyplot as plt
from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from urdf_parser_py.urdf import URDF
from grop import *
from config import *


class RobotGripper(object):
    def __init__(self):
        super(RobotGripper, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place", anonymous=True)
        rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm = self.robot.arm
        self.gripper = self.robot.gripper

        self.world = self.arm.get_planning_frame()
        self.tip = self.arm.get_end_effector_link()

        self.jointsHome = self.arm.get_named_target_values("home")
        self.jointsOpen = self.gripper.get_named_target_values("open")
        self.jointsClose = self.gripper.get_named_target_values("close")

        self.urdf = URDF.from_parameter_server()
        limit = self.urdf.joint_map[self.gripper.get_active_joints()[0]].limit
        self.timeClose = abs(limit.upper - limit.lower) / limit.velocity

        for group in [self.arm, self.gripper]:
            group.set_max_velocity_scaling_factor(1)
            group.set_max_acceleration_scaling_factor(1)

        service(
            globals(),
            ("get_planning_scene", GetPlanningScene),
            ("move_group/save_map", moveit_msgs.srv.SaveMap),
            ("clear_octomap", Empty),
        )

        self.setScene()
        self.home()

    def start(self):
        self.generate_map()
        self.pick_and_place()

    def generate_map(self, *args):
        self.inspect(source)
        self.home()
        self.save_map(*args)

    def pick_and_place(self):
        self.grop()
        self.pick()
        self.place()
        self.home()

    def save_map(self, name="map", path=os.path.join(os.getcwd(), "map")):
        file = os.path.join(path, name + ".bt")
        os.makedirs(os.path.dirname(file), exist_ok=True)

        if save_map(file):
            rospy.loginfo("Map saved")

        else:
            rospy.loginfo("Map saving failed")

    def setAllowedCollision(self, object, link, allowed=True):
        request = PlanningSceneComponents()
        request.components += PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        planning_scene = get_planning_scene(request).scene
        planning_scene.is_diff = True

        try:
            indexObject = planning_scene.allowed_collision_matrix.entry_names.index(
                object
            )
            number = len(planning_scene.allowed_collision_matrix.entry_names)

        except:
            planning_scene.allowed_collision_matrix.entry_names.append(object)
            number = len(planning_scene.allowed_collision_matrix.entry_names)
            indexObject = number - 1
            planning_scene.allowed_collision_matrix.entry_values.append(
                AllowedCollisionEntry([False] * indexObject)
            )

            for i in range(number):
                planning_scene.allowed_collision_matrix.entry_values[i].enabled.append(
                    False
                )

        for i in range(number):
            if link in planning_scene.allowed_collision_matrix.entry_names[i]:
                planning_scene.allowed_collision_matrix.entry_values[i].enabled[
                    indexObject
                ] = allowed
                planning_scene.allowed_collision_matrix.entry_values[
                    indexObject
                ].enabled[i] = allowed

        self.scene.apply_planning_scene(planning_scene)

    def setScene(self):
        self.scene.clear()
        self.setAllowedCollision("desk", "base")

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = self.world
        poseStamped.pose.position = Point()
        poseStamped.pose.orientation = eulerToQuaternion()

        poseStamped.pose.position = Point(*desk.center)
        self.scene.add_box("desk", poseStamped, size=desk_size)

        poseStamped.pose.position = Point(*floor.center)
        self.scene.add_box("floor", poseStamped, size=floor_size)

        # self.scene.add_plane("surface", poseStamped)
        # self.arm.set_support_surface_name("surface")
        # self.arm.set_workspace(np.concatenate((space.min[0:2], space.max[0:2])))

    def go(
        self,
        joints=None,
        end_effector_link=None,
        wait=True,
        eef_step=0.01,
        jump_threshold=0,
    ):
        if end_effector_link:
            end_effector_link_old = self.arm.get_end_effector_link()
            self.arm.set_end_effector_link(end_effector_link)

        if type(joints) == list:
            if type(joints[0]) == PoseStamped:
                pose_reference_frame_old = self.arm.get_pose_reference_frame()
                self.arm.set_pose_reference_frame(joints[0].header.frame_id)
                joints = [joint.pose for joint in joints]

            plan, fraction = self.arm.compute_cartesian_path(
                joints, eef_step, jump_threshold
            )
            self.arm.execute(plan, wait)

            if type(joints[0]) == PoseStamped:
                self.arm.set_pose_reference_frame(pose_reference_frame_old)

        else:
            self.arm.go(joints, wait)

        if end_effector_link:
            self.arm.set_end_effector_link(end_effector_link_old)

    def grip(self, close, wait=True):
        if close:
            joints = self.jointsClose
        else:
            joints = self.jointsOpen
        self.gripper.go(joints, wait)

    def home(self):
        self.go(self.jointsHome)
        self.grip(0)

    def inspect(self, space, times=10):
        positions, targets = (
            [
                space[x, y, z]
                for x, y in [
                    (1 / 4, 1 / 4),
                    (1 / 4, 3 / 4),
                    (3 / 4, 3 / 4),
                    (3 / 4, 1 / 4),
                ]
            ]
            for z in [1, 0]
        )

        self.poses = lookAtPath(positions, targets)

        for _ in range(times):
            self.go(self.poses, "camera_link")

    def grop(self):
        gp = Grop(gripper, space(0))
        position, rotation = gp.optimize()
        self.posePick = PoseStamped()
        self.posePick.header.frame_id = self.world
        self.posePick.pose = matrixToPose(position, rotation)
        self.scene.add_box("part", self.posePick, size=3 * (0.05,))
        clear_octomap()

    def jointTrajectory(self, target, time):
        jointTrajectory = JointTrajectory()
        jointTrajectory.joint_names = list(target.keys())
        jointTrajectory.points = [
            JointTrajectoryPoint(
                positions=list(target.values()), time_from_start=genpy.Duration(t)
            )
            for t in [time]
        ]
        return jointTrajectory

    def pick(self):
        self.grasp = Grasp()
        self.grasp.allowed_touch_objects = ["part", "desk"]
        self.grasp.grasp_pose = self.posePick

        self.grasp.pre_grasp_approach.direction.header.frame_id = self.world
        self.grasp.pre_grasp_approach.direction.vector.z = -1
        self.grasp.pre_grasp_approach.min_distance = d_f[2]
        self.grasp.pre_grasp_approach.desired_distance = (
            2 * self.grasp.pre_grasp_approach.min_distance
        )

        self.grasp.post_grasp_retreat = copy.deepcopy(self.grasp.pre_grasp_approach)
        self.grasp.post_grasp_retreat.direction.vector.z = 1

        self.grasp.pre_grasp_posture = self.jointTrajectory(
            self.jointsOpen, self.timeClose
        )
        self.grasp.grasp_posture = self.jointTrajectory(
            self.jointsClose, self.timeClose
        )

        self.arm.pick("part", self.grasp)

    def place(self, layout="m", n=10, m=10):
        self.placeLocations = []
        placeLocation = PlaceLocation()
        placeLocation.allowed_touch_objects = copy.deepcopy(
            self.grasp.allowed_touch_objects
        )
        placeLocation.pre_place_approach = copy.deepcopy(self.grasp.pre_grasp_approach)
        placeLocation.post_place_retreat = copy.deepcopy(self.grasp.post_grasp_retreat)
        placeLocation.post_place_posture = self.jointTrajectory(
            self.jointsOpen, self.timeClose
        )
        placeLocation.place_pose = copy.deepcopy(self.posePick)

        if layout.startswith("g"):  # grid
            for i in range(n):
                for j in range(m):
                    placeLocation = copy.deepcopy(placeLocation)
                    position = space[(1 + i / n) / 2, (1 + j / m) / 2]
                    placeLocation.place_pose.pose.position.x = position[0]
                    placeLocation.place_pose.pose.position.y = position[1]
                    self.placeLocations.append(placeLocation)
        else:
            if layout.startswith("m"):  # mirror else same
                placeLocation.place_pose.pose.position.y += (
                    (-1) ** (placeLocation.place_pose.pose.position.y > space.center[1])
                    * space.size[1]
                    / 2
                )
            self.placeLocations.append(placeLocation)

        self.arm.place("part", self.placeLocations)
        self.scene.remove_world_object("part")


if __name__ == "__main__":
    try:
        robot = RobotGripper()
        robot.generate_map()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass
