#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.srv
from std_msgs.msg import *
from geometry_msgs.msg import *
from moveit_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.srv import *
from std_srvs.srv import *
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

    def cycle(self, **kwargs):
        self.find_pose()
        self.pick_and_place(**kwargs)

    def find_pose(self):
        self.generate_map()
        self.generate_pose()

    def generate_map(self, name="map", times=1):
        self.inspect(source, times)
        self.home()
        self.save_map(name)

    def pick_and_place(self, **kwargs):
        self.pick(**kwargs)
        self.place()
        self.home()

    def save_map(self, name="map"):
        if save_map_service(map_file(name)):
            rospy.loginfo("Map saved")

        else:
            rospy.loginfo("Map saving failed")

    def setAllowedCollision(self, object, link, allowed=True):
        request = PlanningSceneComponents()
        request.components += PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        planning_scene = get_planning_scene_service(request).scene
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

        poseStamped.pose.position = Point(*desk.get_center())
        self.scene.add_box("desk", poseStamped, size=desk_size)

        poseStamped.pose.position = Point(*floor.get_center())
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

    def inspect(self, space, times=1):
        positions, targets = (
            [
                space[x, y, z]
                for x, y in [
                    (1 / 4, 1 / 8),
                    (1 / 4, 7 / 8),
                    (3 / 4, 7 / 8),
                    (3 / 4, 1 / 8),
                ]
            ]
            for z in [1, 0]
        )

        self.poses = lookAtPath(positions, targets)

        for _ in range(times):
            self.go(self.poses, "camera_link")

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

    def generate_pose(self):
        gp = Grop(gripper, objects)
        objects.set_map()
        pose = gp.optimize()
        self.posePick = PoseStamped()
        self.posePick.header.frame_id = self.world
        self.posePick.pose = pose.pose()
        self.scene.add_box("object", self.posePick, size=3 * (0.05,))
        clear_octomap_service()
        return self.posePick

    def pick(self, test=False):
        self.grasp = Grasp()
        self.grasp.allowed_touch_objects = ["object", "desk"]
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
        self.grasp.grasp_posture = (
            self.grasp.pre_grasp_posture
            if test
            else self.jointTrajectory(self.jointsClose, self.timeClose)
        )

        self.arm.pick("object", self.grasp)

    def place(self, layout="g", n=10, m=10):
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
                    (-1)
                    ** (
                        placeLocation.place_pose.pose.position.y > space.get_center()[1]
                    )
                    * space.size[1]
                    / 2
                )

            self.placeLocations.append(placeLocation)

        self.arm.place("object", self.placeLocations)
        self.scene.remove_world_object("object")


if __name__ == "__main__":
    try:
        robot = RobotGripper()
        robot.cycle()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass
