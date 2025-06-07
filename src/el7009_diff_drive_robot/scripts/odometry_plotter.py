#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class OdometryPlotter(Node):

    def __init__(self):
        super().__init__('odometry_plotter')

        self.pub = self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
       
        self._loop_rate = self.create_rate(10, self.get_clock())
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.positions = []

    def listener_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        _,_,self.theta = euler_from_quaternion(orientation)
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
        
    
def main(args=None):
    rclpy.init(args=args)
    plotter = OdometryPlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info('Nodo detenido manualmente con Ctrl+C')
    finally:
        plotter.plot_odometry()  # Aquí graficamos al salir
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

