#!/usr/bin/env python3

import os
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import builtin_interfaces.msg
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _set_pose(frame_id: str, timestamp: builtin_interfaces.msg.Time, xy: list, quat: list) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = timestamp
    pose.pose.position.x = float(xy[0])
    pose.pose.position.y = float(xy[1])
    pose.pose.position.z = 0.0

    pose.pose.orientation.w = float(quat[0])
    pose.pose.orientation.x = float(quat[1])
    pose.pose.orientation.y = float(quat[2])
    pose.pose.orientation.z = float(quat[3])

    return pose


def load_waypoints_from_file(frame_id: str, timestamp: builtin_interfaces.msg.Time, file_path: str):
    waypoints = []

    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    for wp_name, wp_data in data['waypoints'].items():
        pose = wp_data.get("pose")
        orientation = wp_data.get("orientation")
        waypoint = _set_pose(frame_id, timestamp, pose, orientation)
        waypoints.append(waypoint)

    return waypoints


def main(args=None):
    rclpy.init(args=args)

    yaml_path = os.path.join(
        get_package_share_directory('gizmo_mapping'),
        'maps', 'savedWaypointPose', 'justSimpleBiggerWorldPose.yaml'
    )

    nav = BasicNavigator()

    # -------------------------------------------------------------------------
    # Localizer Lifecycle Synchronization:
    # - localizer='amcl': Default for pre-built map navigation (mode:=localization).
    #   BasicNavigator waits for the /amcl/get_state lifecycle service.
    # - localizer='slam_toolbox': Used for online SLAM mapping navigation (mode:=mapping).
    #   BasicNavigator waits for the /slam_toolbox/get_state lifecycle service.
    # -------------------------------------------------------------------------
    nav.waitUntilNav2Active(localizer='amcl')
    # For online SLAM mapping navigation:
    # nav.waitUntilNav2Active(localizer='slam_toolbox')

    waypoints = load_waypoints_from_file("map", nav.get_clock().now().to_msg(), yaml_path)
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Navigating to waypoint: [{feedback.current_waypoint + 1}/{len(waypoints)}]", end='\r', flush=True)

    print()

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info("Goal succeeded! Gizmo reached all target waypoints.")
    elif result == TaskResult.CANCELED:
        nav.get_logger().warning("Goal was canceled.")
    elif result == TaskResult.FAILED:
        nav.get_logger().error("Goal failed! Gizmo could not reach the target destination.")
    else:
        nav.get_logger().info(f"Goal returned status code: {result}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
