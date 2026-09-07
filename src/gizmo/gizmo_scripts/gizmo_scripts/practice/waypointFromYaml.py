#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import builtin_interfaces.msg
from tf_transformations import quaternion_from_euler
import yaml

def _set_pose(_frame_id: str, _timeStamp: builtin_interfaces.msg.Time, x: float, y: float, quat: list) -> PoseStamped:
    _pose = PoseStamped()
    _pose.header.frame_id = _frame_id
    _pose.header.stamp = _timeStamp
    _pose.pose.position.x = x
    _pose.pose.position.y = y
    _pose.pose.position.z = 0.0

    # In YAML: [w, x, y, z] -> directly assign to PoseStamped
    _pose.pose.orientation.w = float(quat[0])
    _pose.pose.orientation.x = float(quat[1])
    _pose.pose.orientation.y = float(quat[2])
    _pose.pose.orientation.z = float(quat[3])

    return _pose

def load_waypoints_from_file(_frame_id: str, _timeStamp: builtin_interfaces.msg.Time, file_path: str):
    waypoints = []

    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    for wp_name, wp_data in data['waypoints'].items():
        pose = wp_data.get("pose")                # [x, y, z]
        orientation = wp_data.get("orientation")  # [w, x, y, z]
        
        # Pass the whole orientation list directly!
        waypoint = _set_pose(_frame_id, _timeStamp, pose[0], pose[1], orientation)
        waypoints.append(waypoint)

    return waypoints

def main(args=None):
    # Absolute path to the saved waypoint YAML file
    yaml_path = "/root/ros2_ws/src/gizmo/gizmo_navigation/maps/savedWaypointPose/test_waypoint.yaml"
    # yaml_path = "/root/ros2_ws/src/gizmo/gizmo_navigation/maps/savedWaypointPose/justSimpleBiggerWorldPose.yaml"

    rclpy.init(args=args)

    nav = BasicNavigator()
    # nav.cancelTask()  # Cancel any existing navigation tasks

    nav.waitUntilNav2Active(localizer='slam_toolbox')
    # waypoints = load_waypoints_from_file(yaml_path)
    

    # nav.followWaypoints([_set_pose("map", nav.get_clock().now().to_msg(), 2.396, 0.394, 0.348),
    #                     _set_pose("map", nav.get_clock().now().to_msg(), 1.611, 2.829, 0.978),
    #                     _set_pose("map", nav.get_clock().now().to_msg(), -0.652, 2.721, 0.257)])
    waypoints = load_waypoints_from_file("map", nav.get_clock().now().to_msg(), yaml_path)
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Navigating to waypoint: [{feedback.current_waypoint + 1}/{len(waypoints)}]", end='\r', flush=True)
        # if feedback:
        #     print(f"Distance remaining: {feedback.distance_remaining:.2f} m | Navigation time: {feedback.navigation_time.sec} s", end='\r')

    print()  # New line after feedback loop

    # Check Result
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