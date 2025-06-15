#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
            10
        )
        self.estimation_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.estimation_callback,
            10
        )
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
        self.ground_truth = []
        self.estimation = []

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
        self.ground_truth.append([self.x, self.y, self.theta])
    def estimation_callback(self, msg:PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        _,_,self.theta = euler_from_quaternion(orientation)
        self.estimation.append([self.x, self.y, self.theta])


    def joy_callback(self,msg):  
        if msg.buttons[7] == 1 and not self.plot:
            self.plot = True 
            self.plot_odometry()
    def plot_odometry(self):
        gt_x, gt_y, gt_theta = zip(*self.ground_truth)  # Descomprimir listas x e y
        est_x, est_y, est_theta = zip(*self.estimation)

        plt.figure(figsize=(6, 6))
        plt.plot(gt_x, gt_y, label="Trayectoria real del robot")
        plt.plot(est_x, est_y, label='Trayectoria según AMCL (sin calibrar)')        
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Trayectoria real vs AMCL sin calibrar")
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
