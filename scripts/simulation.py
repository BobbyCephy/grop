#!/usr/bin/env python
import subprocess

p = subprocess.Popen(
    "gnome-terminal -e 'roslaunch robot_gripper_camera world.launch'", shell=True
)
