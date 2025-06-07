#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import threading
from std_msgs.msg import Float64
import numpy as np
from random import choice
from nav_msgs.msg import Odometry
import json


import os

file_path = os.path.expanduser('~/EL7009/tarea_1/el7009_diff_drive_robot/scripts/variables.json')

with open(file_path, 'r') as json_file:
    data = json.load(json_file)


_l = data["_l"]
_r = data["_r"]
_phi = data["_phi"]

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
class ExampleControllerMinimal(Node):

    def __init__(self):
        super().__init__('Inverse_Kinematics')
        self.publisher_left_ = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self.publisher_right_ = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        self.pub = self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.odom = None
        self.update_rate = 10
        self._loop_rate = self.create_rate(self.update_rate, self.get_clock())
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.positions = []

    def listener_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        self.positions.append((self.x, self.y))
    
    def plot_odometry(self):
        if not self.positions:
            self.get_logger().info("No hay datos de odometría para graficar.")
            return
        
        x_vals, y_vals = zip(*self.positions)  # Descomprimir listas x e y
        
        plt.figure(figsize=(6, 6))
        plt.plot(x_vals, y_vals, marker='o', linestyle='-', color='b', label="Trayectoria")
        plt.scatter(x_vals[0], y_vals[0], color='g', label="Inicio", marker="o")  # Punto inicial
        plt.scatter(x_vals[-1], y_vals[-1], color='r', label="Final", marker="x")  # Punto final
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Trayectoria del Robot")
        plt.legend()
        plt.grid()
        plt.show()
        
    def atan2(self,pose):
        """ calculates the angle that the robot should rotate in order
        to allow it to travel in a straight path to a given position.
        """
        x, y, theta = pose
        return np.arctan2(y,x)

    def rotate(self,pose, l = _l, r = _r, phi = _phi):
        """ returns the amount of time that each motor has to be active, 
        given the geometry of the differential robot and its angular speed,
        to achieve a certain orientation
        """
        x,y,theta = pose
        return abs((l*theta)/(2*r*phi))

    def go(self,pose, l = _l, r = _r, phi = _phi):
        """ returns the amount of time that each motor has to be active, 
        given the geometry of the differential robot and its angular speed,
        to move forward a certain distance
        """
        x,y,theta = pose
        return abs(x/(r*phi))

    # def direction(self,theta):
    #     """ function to decide the best turning direction (left or right)
    #     when it has to rotate to np.pi, it chooses randomly to rotate left or right
    #     """
    #     if theta == np.pi or theta == -np.pi:
    #         return choice((1,-1))
    #     else:
    #         return np.sign(theta)
    



    def IK(self, pf, l = _l, r = _r, phi = _phi):
        """ Moves a differential robot to a pose
        """    
        msg = Float64()
        # Spin in a separate thread so that the clock is updated


        # look planning phase
        goal_yaw = self.atan2(pf)
        dir_l = np.sign(goal_yaw)
        x, y, theta = pf
        d = np.sqrt(x**2 + y**2)
        look = (d, 0.0, goal_yaw)
        
        dt_l = self.rotate(look)
        looking_time = int(dt_l * self.update_rate)

        # moving planning phase
        dt_m = self.go(look)
        moving_time = int(dt_m * self.update_rate)

        # reorientation planning phase
        theta_2 = theta - goal_yaw
        dir_r = np.sign(theta_2)
        new_pose = (0, 0, theta_2) 
        dt_r = self.rotate(new_pose)
        reorientation_time = int(dt_r * self.update_rate)


        # Movement phase
        if dir_l != 0:
            for i in range(looking_time): 
                msg.data = (-1)*dir_l*phi
                self.publisher_left_.publish(msg)
                msg.data = dir_l*phi
                self.publisher_right_.publish(msg)
                self._loop_rate.sleep()
        msg.data = 0.0
        self.publisher_left_.publish(msg)
        self.publisher_right_.publish(msg)
        
        if d > 0.001:
            for i in range(moving_time): 
                msg.data = phi
                self.publisher_left_.publish(msg)
                self.publisher_right_.publish(msg)
                self._loop_rate.sleep()
        msg.data = 0.0
        self.publisher_left_.publish(msg)
        self.publisher_right_.publish(msg)


        if dir_r != 0:
            for i in range(reorientation_time): 
                msg.data = (-1)*dir_r*phi
                self.publisher_left_.publish(msg)
                msg.data = dir_r*phi
                self.publisher_right_.publish(msg)
                self._loop_rate.sleep()
        msg.data = 0.0
        self.publisher_left_.publish(msg)
        self.publisher_right_.publish(msg)

        self.get_logger().info(f"Arrived to (x = {pf[0]:.2f}, y = {pf[1]:.2f}, theta = {pf[2]:.2f})!")
    
def main(args=None):
    rclpy.init(args=args)
    controller = ExampleControllerMinimal()
    thread = threading.Thread(target=rclpy.spin, args=(controller,))
    thread.start()
    # controller.IK((-2.3,-1.0,-np.pi))

    waypoints = [
    ( -3.84,  0.00,  0.0),
    ( -3.84,  3.76,  0.0),
    (  0.00,  3.76,  0.0),
    (  0.00,  0.00,  0.0) 
    ]


    for target in waypoints:
        x_actual, y_actual, theta_actual = controller.x, controller.y, controller.theta
        print(f'{x_actual = :.2f}, {y_actual = :.2f}, {theta_actual = :.2f}')
        x_objetivo = target[0] - x_actual
        y_objetivo = target[1] - y_actual
        theta_objetivo = target[2] - theta_actual
        
        controller.IK((x_objetivo, y_objetivo, theta_objetivo))
    controller.plot_odometry()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

