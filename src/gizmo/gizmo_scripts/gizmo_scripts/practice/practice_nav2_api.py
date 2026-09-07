#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import builtin_interfaces.msg
from tf_transformations import quaternion_from_euler
import yaml

def _set_pose(_frame_id: str, _timeStamp, x: float, y: float, yaw: float) -> PoseStamped:

    _timeStamp: builtin_interfaces.msg.Time = _timeStamp

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

    rclpy.init(args=args)

    nav = BasicNavigator()

    nav.waitUntilNav2Active(localizer='slam_toolbox')
    # waypoints = load_waypoints_from_file(yaml_path)

    nav.followWaypoints([_set_pose("map", nav.get_clock().now().to_msg(), 2.396, 0.394, 0.348),
                        _set_pose("map", nav.get_clock().now().to_msg(), 1.611, 2.829, 0.978),
                        _set_pose("map", nav.get_clock().now().to_msg(), -0.652, 2.721, 0.257)])
                        
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
        # if feedback:
        #     print(f"Distance remaining: {feedback.distance_remaining:.2f} m | Navigation time: {feedback.navigation_time.sec} s", end='\r')

    print()  # New line after feedback loop

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

if __name__ == "__main__":
    main()