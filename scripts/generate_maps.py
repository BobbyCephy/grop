#!/usr/bin/env python
import os
import rospkg
import xacro
from gazebo_msgs.srv import *
from config import *
from utils import *

package_path = rospkg.RosPack().get_path("robot_gripper_camera")
part_path = os.path.join(package_path, "part")
mesh_path = os.path.join(part_path, "mesh")
mesh_files = os.listdir(mesh_path)


def xacro_part(model):
    return os.path.join(part_path, model + ".urdf.xacro")


def urdf_string(xacro_file, **kwargs):
    return xacro.process_file(xacro_file, mappings=kwargs).toxml()


def urdf_part(model, **kwargs):
    return urdf_string(xacro_part(model), **kwargs)


for _ in range(2):
    try:
        service(
            globals(),
            ("/gazebo/spawn_sdf_model", SpawnModel),
            ("/gazebo/delete_model", DeleteModel),
        )

    except:
        import simulation


def delete(name):
    delete_model(DeleteModelRequest(model_name=name))


def spawn(model, pose, name, **kwargs):
    spawn_sdf_model(
        model_name=name,
        model_xml=urdf_part(model, name=name, **kwargs),
        initial_pose=pose,
        reference_frame="robot",
    )


def spawn_cuboid(size, pose, suffix=""):
    pose.position.z += size[2] / 2
    model = "cuboid"
    name = model + suffix

    spawn(
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
    spawn("mesh", pose, name, file=file)
    return name


from pick_and_place import *

robot = RobotGripper()


def cuboids(size, n=1, m=None):
    if m is None:
        m = n

    names = []

    for i in range(n):
        for j in range(m):
            position = source[(i + 1 / 2) / n, (j + 1 / 2) / m, 0]
            names.append(spawn_cuboid(size, pose(position), str(i) + str(j)))

    if n == 1 and m == 1:
        map_name = "cuboid" + "/" + "x".join(str(s) for s in size)

    else:
        map_name = "grid" + "/" + str(n) + "x" + str(m)

    robot.generate_map(map_name)

    for name in names:
        delete(name)


size = np.array([0.5, 1, 2]) * d_olim[1]


def cuboid():
    for y in range(1, 10):
        cuboids([size[0], size[0] * y, size[2]])


def grid():
    for x in range(1, 5):
        for y in range(1, 5):
            cuboids(size, x, y)


def meshes():
    for file in mesh_files:
        file_path = os.path.join(mesh_path, file)
        position = source[1 / 2, 1, 0]
        name = spawn_mesh(file_path, pose(position))

        robot.generate_map("mesh" + "/" + name)
        delete(name)


if __name__ == "__main__":
    grid()
