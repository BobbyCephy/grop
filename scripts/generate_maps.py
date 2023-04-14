#!/usr/bin/env python
import os
import rospkg
import xacro
from gazebo_msgs.srv import *
from config import *
from utils import *

package_path = rospkg.RosPack().get_path("robot_gripper_camera")
object_path = os.path.join(package_path, "object")
mesh_path = os.path.join(object_path, "mesh")
mesh_files = os.listdir(mesh_path)


def xacro_object(model):
    return os.path.join(object_path, model + ".urdf.xacro")


def urdf_string(xacro_file, **kwargs):
    return xacro.process_file(xacro_file, mappings=kwargs).toxml()


def urdf_object(model, **kwargs):
    return urdf_string(xacro_object(model), **kwargs)


for _ in range(2):
    try:
        service(
            globals(),
            ("/gazebo/spawn_sdf_model", SpawnModel),
            ("/gazebo/delete_model", DeleteModel),
        )

    except:
        import simulation


def delete_model(*names):
    for name in names:
        delete_model_service(DeleteModelRequest(model_name=name))


def spawn_model(model, pose, name, **kwargs):
    spawn_sdf_model_service(
        model_name=name,
        model_xml=urdf_object(model, name=name, **kwargs),
        initial_pose=pose,
        reference_frame="robot",
    )


def spawn_cuboid(size, pose, suffix=""):
    pose.position.z += size[2] / 2
    model = "cuboid"
    name = model + suffix

    spawn_model(
        model,
        pose,
        name,
        x=str(size[0]),
        y=str(size[1]),
        z=str(size[2]),
    )

    return name


def spawn_mesh(file, pose):
    name = os.path.splitext(os.path.basename(file))[0]
    spawn_model("mesh", pose, name, file=file)
    model_names.append(name)
    return name


from pick_and_place import *

robot = RobotGripper()
model_names = []
size = np.array([0.5, 1, 2]) * d_olim[1]

def generate_map(map_name, model_names, wait=False):
    robot.generate_map(map_name, 5)
    if wait:
        input()
    delete_model(*model_names)


def spawn_cuboids(size=size, number=1):
    number = np.resize(number, 3)

    for index in np.ndindex(*number):
        rel = (np.array(index) + 1 / 2) / number
        rel[2] = 0
        position = source[rel]
        position[2] += index[2] * size[2]
        model_names.append(
            spawn_cuboid(size, pose(position), "x".join(map(str, index)))
        )

    map_name = "_".join(["cuboid", *map(join_x, [size, number])])

    return map_name, model_names


def map_cuboids(*args, **kwargs):
    generate_map(*spawn_cuboids(*args, **kwargs))


def map_meshes():
    for file in mesh_files:
        file_path = os.path.join(mesh_path, file)
        position = source[1 / 2, 1, 0]
        map_name = spawn_mesh(file_path, pose(position))
        robot.generate_map(map_name, map_name)


def cuboid():
    for y in range(1, 10):
        map_cuboids([size[0], size[0] * y, size[2]])


def grid():
    for x in range(1, 5):
        for y in range(1, 5):
            map_cuboids(size, x, y)


if __name__ == "__main__":
    map_cuboids(size, (1, 3, 1))
