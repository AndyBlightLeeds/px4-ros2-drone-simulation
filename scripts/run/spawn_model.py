#!/usr/bin/env python3
"""
Copyright 2020 University of Leeds.
Distributed under the Apache 2.0 license.

Based on https://github.com/osrf/drone_demo launch_drone_ros2.py and
https://answers.ros.org/question/314607/spawn-model-with-ros2-gazebo/
"""

import math
import os
import subprocess
import sys

import rclpy
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def spawn_model(
    node,
    model_name,
    model_xml,
    pose,
    ros_master_uri=None,
    robot_namespace=None,
    debug=True,
    service_name="/spawn_entity",
):
    x, y, yaw = pose
    INITIAL_HEIGHT = 0.1  # m
    node.get_logger().info("spawn_model called")
    client = node.create_client(SpawnEntity, service_name)

    while not client.wait_for_service(timeout_sec=1.0):
        print(service_name, "service not available, waiting again...")

    if debug:
        print(model_xml)

    req = SpawnEntity.Request()
    req.name = model_name
    req.xml = model_xml
    req.robot_namespace = robot_namespace if robot_namespace else model_name
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = INITIAL_HEIGHT
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = math.sin(yaw / 2.0)
    req.initial_pose.orientation.w = math.cos(yaw / 2.0)
    req.reference_frame = ""
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is not None:
        print(resp.status_message, "(%s)" % model_name)
        return 0
    else:
        print(resp.status_message, file=sys.stderr)
        return 1


def main():
    rclpy.init(args=None)
    node = rclpy.create_node("spawn_models")
    # Hard code the params.
    drone_type = "iris"
    vehicle_id = 0
    vehicle_name = drone_type + "_%s" % vehicle_id
    pose = (0.0, 0.0, 0.0)
    # Create file name using environment variable PX4_SITL_GAZEBO_GIT_DIR made
    # available by the calling Bash script.
    firmware_dir = os.environ['PX4_SITL_GAZEBO_GIT_DIR']
    sdf_file_name = firmware_dir + "/models/iris/iris.sdf"
    # Load XML from file.
    xml = ""
    with open(sdf_file_name, 'r') as sdf_file:
        xml = sdf_file.read()
    # Finally spawn the model.
    spawn_model(node, vehicle_name, xml, pose, debug=False)
    print("model ", vehicle_name, "spawned.")


if __name__ == "__main__":
    main()
