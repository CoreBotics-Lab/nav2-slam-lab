#!/usr/bin/env python3

"""
Autonomous Mission Script for Gizmo using Nav2 Simple Commander API (SLAM Mode).

How to run:
  ros2 run gizmo_scripts go_to_goal_slam_toolbox -x 1.0
  ros2 run gizmo_scripts go_to_goal_slam_toolbox -x 1.5 -y 0.5 -yaw 1.57
  ros2 run gizmo_scripts go_to_goal_slam_toolbox --help
"""

import argparse
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


def main(args=None):
    # 1. Parse CLI arguments using standard argparse
    parser = argparse.ArgumentParser(description='Send Gizmo to a goal pose using Nav2')
    parser.add_argument('-x', '--x', type=float, default=2.0, help='Target X coordinate in meters (default: 2.0)')
    parser.add_argument('-y', '--y', type=float, default=0.0, help='Target Y coordinate in meters (default: 0.0)')
    parser.add_argument('-yaw', '--yaw', type=float, default=0.0, help='Target Yaw orientation in radians (default: 0.0)')

    # parse_known_args parses -x, -y, --yaw while ignoring any ROS-specific flags
    parsed_args, _ = parser.parse_known_args()
    x = parsed_args.x
    y = parsed_args.y
    yaw = parsed_args.yaw

    rclpy.init(args=args)
    nav = BasicNavigator()
    nav.get_logger().info(f"Target Goal Position: X={x:.2f} m, Y={y:.2f} m, Yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)")

    # 2. Configured for slam_navigation.launch.py:
    # Since slam_navigation uses slam_toolbox instead of amcl, we pass localizer='slam_toolbox'.
    # Note: If running navigation.launch.py (with a static map), use localizer='amcl'.
    nav.waitUntilNav2Active(localizer='slam_toolbox')

    # 3. Define Goal Pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0

    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w

    # 4. Send the Goal
    nav.goToPose(goal_pose)

    # 5. Monitor Progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} m | Navigation time: {feedback.navigation_time.sec} s", end='\r')

    print()  # New line after feedback loop

    # 6. Check Result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info("Goal succeeded! Gizmo reached the target destination.")
    elif result == TaskResult.CANCELED:
        nav.get_logger().warning("Goal was canceled.")
    elif result == TaskResult.FAILED:
        nav.get_logger().error("Goal failed! Gizmo could not reach the target destination.")
    else:
        nav.get_logger().info(f"Goal returned status code: {result}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
