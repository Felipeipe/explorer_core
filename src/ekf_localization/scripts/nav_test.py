#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion
import os

class OdometryPlotter(Node):

    def __init__(self):
        super().__init__('odometry_plotter')

        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.gt_callback,
            10)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        ) 
        self._loop_rate = self.create_rate(10, self.get_clock())
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.plot = False
        self.positions = []

    def gt_callback(self,msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        _,_,self.theta = euler_from_quaternion(orientation)
        self.positions.append([self.x, self.y, self.theta])

    def joy_callback(self,msg):  
        if msg.buttons[7] == 1 and not self.plot:
            self.plot = True 
            self.plot_odometry()
    def plot_odometry(self):
        if not self.positions:
            self.get_logger().info("No hay datos de odometría para graficar.")
            return
        x_vals, y_vals, _ = zip(*self.positions)  # Descomprimir listas x e y
        
        plt.figure(figsize=(6, 6))
        plt.plot(x_vals, y_vals, color='b', label="Trayectoria")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Trayectoria del Robot")
        plt.legend(loc='best')
        plt.grid(True)
        plt.savefig('Ground_truth_trajectory.png')
        self.get_logger().info(f'Plotted correctly, figure saved in {os.getcwd()}')
        self.plot = False    
        
    
def main(args=None):
    rclpy.init(args=args)
    plotter = OdometryPlotter()
    plotter.get_logger().info('Node started successfully')
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
