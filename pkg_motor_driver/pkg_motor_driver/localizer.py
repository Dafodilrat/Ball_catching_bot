import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from Rosmaster_Lib import Rosmaster
import serial
from sensor_msgs.msg import Imu,MagneticField
from time import time

class localizer(Node):

    def __init__(self):
        super().__init__('localizer')  
        self.subscription = self.create_subscription(
            Imu, 
            '/rosmaster/imu_data',  
            self.imu_callback, 
            10  
        )
        self.position_publisher = self.create_publisher(Point, '/rosmaster/pose', 10)

        self.vx=0;self.vy=0;self.vz=0

        self.start_time=time()


    def imu_callback(self,imu):

        dt=time()-self.start_time

        # Fill IMU linear acceleration
        x=imu.linear_acceleration.x 
        y=imu.linear_acceleration.y 
        z=imu.linear_acceleration.z 

        self.vx += x * dt
        self.vy += y * dt
        self.vz += z * dt

        position=Point()

        position.x += self.vx * dt
        position.y += self.vy * dt
        position.z += self.vz * dt

        self.position_publisher.publish(position)


def main(args=None):
    rclpy.init(args=args)  
    node = localizer()  
    rclpy.spin(node)  
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()