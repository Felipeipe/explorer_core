#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf_transformations import quaternion_from_euler
from random import uniform
from numpy import pi

class GoalPosePublisher(Node):

    def __init__(self,x,y,theta):
        super().__init__('goal_pose_publisher')


        # Obtener parámetros
        self._x = x
        self._y = y
        self._theta = theta
        self.publisher_goal = self.create_publisher(PoseStamped, '/move_base_simple/goal',10)
        self.timer = self.create_timer(2.0, self.callback)
    
    def callback(self):
        goal_pose = PoseStamped()
        orientation = Quaternion()
        position = Point()

        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(0,0,self._theta)
        position.x, position.y, position.z = self._x, self._y, 0.0

        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "odom"
        goal_pose.pose.position = position
        goal_pose.pose.orientation = orientation

        self.publisher_goal.publish(goal_pose)
        self.timer.cancel()


def main(args = None):
    rclpy.init(args=args)
    x = 10 * uniform(-1,1)
    y = 10 * uniform(-1,1)
    theta = pi * uniform(-1,1)
    gpp = GoalPosePublisher(x,y,theta)
    gpp.get_logger().info("Started GoalPosePublisher node successfully")
    gpp.get_logger().info(f"Going to {x = :.2f}, {y = :.2f}, {theta = :.2f}")
    rclpy.spin_once(gpp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()