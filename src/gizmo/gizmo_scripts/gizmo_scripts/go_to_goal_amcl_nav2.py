#!/usr/bin/env python3
"""
Autonomous Mission Script for Gizmo using Nav2 Simple Commander API (AMCL Static Map Mode).

How to run:
  ros2 run gizmo_scripts go_to_goal_amcl_nav2 -x 1.0
  ros2 run gizmo_scripts go_to_goal_amcl_nav2 -x 1.5 -y 0.5 -yaw 1.57
  ros2 run gizmo_scripts go_to_goal_amcl_nav2 -x 2.0 -y 1.0 --init-x 0.0 --init-y 0.0
  ros2 run gizmo_scripts go_to_goal_amcl_nav2 --help
"""

import argparse
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def _set_pose(_frame_id: str, _timeStamp, x: float, y: float, yaw: float) -> PoseStamped:
    import builtin_interfaces.msg
    _timeStamp: builtin_interfaces.msg.Time = _timeStamp
    from tf_transformations import quaternion_from_euler

    _pose = PoseStamped()
    _pose.header.frame_id = _frame_id
    _pose.header.stamp = _timeStamp
    _pose.pose.position.x = x
    _pose.pose.position.y = y
    _pose.pose.position.z = 0.0

    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)
    _pose.pose.orientation.x = q_x
    _pose.pose.orientation.y = q_y
    _pose.pose.orientation.z = q_z
    _pose.pose.orientation.w = q_w

    return _pose

def main(args=None):
    # 1. Parse CLI arguments using standard argparse
    parser = argparse.ArgumentParser(
        description='Send Gizmo to a goal pose using Nav2 Simple Commander API (AMCL Mode)',
        epilog='Example: ros2 run gizmo_scripts go_to_goal_amcl_nav2 -x 1.5 -y 0.5 -yaw 1.57'
    )
    # Goal pose arguments
    parser.add_argument('-x', '--x', type=float, default=2.0, help='Target Goal X coordinate in meters (default: 2.0)')
    parser.add_argument('-y', '--y', type=float, default=0.0, help='Target Goal Y coordinate in meters (default: 0.0)')
    parser.add_argument('-yaw', '--yaw', type=float, default=0.0, help='Target Goal Yaw orientation in radians (default: 0.0)')

    # Initial pose arguments (for AMCL particle cloud seeding)
    parser.add_argument('--init-x', type=float, default=0.0, help='Initial Pose X coordinate in meters (default: 0.0)')
    parser.add_argument('--init-y', type=float, default=0.0, help='Initial Pose Y coordinate in meters (default: 0.0)')
    parser.add_argument('--init-yaw', type=float, default=0.0, help='Initial Pose Yaw orientation in radians (default: 0.0)')

    parsed_args, _ = parser.parse_known_args()
    goal_x = parsed_args.x
    goal_y = parsed_args.y
    goal_yaw = parsed_args.yaw

    init_x = parsed_args.init_x
    init_y = parsed_args.init_y
    init_yaw = parsed_args.init_yaw

    rclpy.init(args=args)
    nav = BasicNavigator()

    # Enable simulation clock synchronization with Gazebo
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    nav.get_logger().info(f"Setting Initial Pose: X={init_x:.2f} m, Y={init_y:.2f} m, Yaw={init_yaw:.2f} rad")
    nav.get_logger().info(f"Target Goal Position: X={goal_x:.2f} m, Y={goal_y:.2f} m, Yaw={goal_yaw:.2f} rad ({math.degrees(goal_yaw):.1f}°)")

    # 2. Wait for Nav2 & AMCL to become active
    nav.waitUntilNav2Active(localizer='amcl')

    # 3. Set Initial Pose for AMCL (Seeds the Monte Carlo Particle Cloud)
    import time
    while nav.count_subscribers('initialpose') == 0:
        time.sleep(0.1)

    initial_pose: PoseStamped = _set_pose('map', nav.get_clock().now().to_msg(), init_x, init_y, init_yaw)
    nav.setInitialPose(initial_pose)
    time.sleep(0.5)  # Allow AMCL to re-seed particle filter

    # 4. Define Goal Pose
    goal_pose: PoseStamped = _set_pose('map', nav.get_clock().now().to_msg(), goal_x, goal_y, goal_yaw)

    # 5. Send the Goal
    nav.goToPose(goal_pose)

    # 6. Monitor Progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} m | Navigation time: {feedback.navigation_time.sec} s", end='\r')

    print()  # New line after feedback loop

    # 7. Check Result
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
