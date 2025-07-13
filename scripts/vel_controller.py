#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64
import numpy as np
import os
import json


class VelocityController(Node):

    def __init__(self,r,l):
        super().__init__('velocity_controller')

        # self._vel_pub = self.create_publisher(JointState, '/joint_states', self._vel_publish)
        self._pub_left = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self._pub_right = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        self._odom_sub = self.create_subscription(Odometry,'/odom',self.robot_pose,10)
        self._goal_sub = self.create_subscription(PoseStamped,'/move_base_simple/goal',self.goal_pose,10)
        self._timer = self.create_timer(0.1, self.vel_publish)
        self._current_pose = np.zeros(3)
        self._target_pose = np.zeros(3)
        self._r = r
        self._l = l
        self._k1 = 0.2 # distance error gain
        self._k2 = 20 # directioning error gain
        self._k3 = 2 # orientation error gain
        self._tolerance = 0.1
        self._orientation_tolerance = 0.6
        self._goal_reached = False    

    def robot_pose(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        
        _,_,theta = euler_from_quaternion(orientation)

        self._current_pose = np.array([x,y,theta])
    def goal_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation = [msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z,
                       msg.pose.orientation.w]
        _,_,theta = euler_from_quaternion(orientation)
        self._target_pose = np.array([x,y,theta])
        self._goal_reached = False     

        

    def angle_normalizer(self,angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def vel_publish(self):
        delta_x, delta_y, delta_theta = self._target_pose - self._current_pose
        theta_r = self._current_pose[2]
        delta_theta = self.angle_normalizer(delta_theta)
        
        heading_error = self.angle_normalizer(np.arctan2(delta_y,delta_x) - theta_r)

        distance_error = np.sqrt(delta_x**2 + delta_y**2)

        angular_vel_component = (self._l/(2*self._r))*(self._k2*heading_error + self._k3*delta_theta)

        linear = (self._k1/self._r)*distance_error
        phi_l = Float64() 
        phi_r = Float64()
        
        if distance_error < self._tolerance and abs(delta_theta) < self._orientation_tolerance:
            if not self._goal_reached:
                self.get_logger().info("Target reached! Stopping the robot.")
                self._goal_reached = True
            
            phi_r.data = 0.0
            phi_l.data = 0.0
            self._pub_left.publish(phi_l)
            self._pub_right.publish(phi_r)
        else:
            self._goal_reached = False  # Si no estamos en la meta, aseguramos bandera falsa

            phi_r.data = angular_vel_component + linear
            phi_l.data = -angular_vel_component + linear
            self._pub_left.publish(phi_l)
            self._pub_right.publish(phi_r)


        

         
def main(args=None):
    rclpy.init(args=args)
    script_dir = os.path.dirname(os.path.realpath(__file__))

    json_path = os.path.join(script_dir, 'variables.json')

    # Leer el archivo
    with open(json_path, 'r') as f:
        data = json.load(f)
    r = data['_r']
    l = data['_l']

    minimal_subscriber = VelocityController(r,l)
    minimal_subscriber.get_logger().info("Started VelocityController node successfully")
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()