#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion

import numpy as np
import os

class LocalizationTester(Node):

    def __init__(self):
        super().__init__('localization_tester')
        self._gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.gt_callback,
            10
        )
        self._estimated_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.estimated_pose_callback,
            10
        )
        self._joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10 
        )
        self.gt_poses = []
        self.est_poses = []
        self._saved = False       
    def gt_callback(self, msg:PoseStamped):
        q = msg.pose.orientation
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])[2]
        # self.get_logger().info(f"GT Pose: x={msg.pose.position.x}, y={msg.pose.position.y}")
        pose = [x, y, theta]
        self.get_logger().info(f'guardando pose {pose}')
        self.gt_poses.append(pose)
    def estimated_pose_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        self.est_poses.append([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta
        ])
    def joy_callback(self, msg: Joy):
        if msg.buttons[7] == 1 and not self._saved:
            self.get_logger().info(f"Positions saved in {os.getcwd()}")
            np.save('gt_poses.npy', np.array(self.gt_poses))
            np.save('est_poses.npy', np.array(self.est_poses))
            self._saved = True

def main(args=None):
    rclpy.init(args=args)
    tester = LocalizationTester()
    tester.get_logger().info('Started data recollection\n Press the start button to save the trajectories')
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()

        
        

if __name__== '__main__':
    main()
