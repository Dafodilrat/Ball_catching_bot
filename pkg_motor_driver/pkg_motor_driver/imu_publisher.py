import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,MagneticField
import random
from Rosmaster_Lib import Rosmaster
import serial


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, '/rosmaster/imu_data',100)  # Topic name: imu_data
        self.mag_publisher = self.create_publisher(MagneticField, '/rosmaster/mag_data',100)  # Topic name: imu_data
        self.timer = self.create_timer(1/20, self.imu_callback)
        
        self.bot = Rosmaster(com="/dev/ttyUSB0",debug=True)
        self.bot.create_receive_threading()

        # self.bot.ser = serial.Serial("/dev/ttyUSB0",9600)

        
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('car_type', 'X3')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value


    def imu_callback(self):
        imu = Imu()
        mag = MagneticField()

        # Fill header
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_link

        # Fill IMU orientation (quaternion)
        ax, ay, az = self.bot.get_accelerometer_data()
        gx, gy, gz = self.bot.get_gyroscope_data()
        mx, my, mz = self.bot.get_magnetometer_data()

        # Fill IMU angular velocity
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0

        # Fill IMU linear acceleration
        imu.linear_acceleration.x = (ax*1.0)+0.35
        imu.linear_acceleration.y = ay-0.03
        imu.linear_acceleration.z = az-9.25
        imu.angular_velocity.x = gx*1.0
        imu.angular_velocity.y = gy*1.0
        imu.angular_velocity.z = gz*1.0

        mag.magnetic_field.x = mx*1.0
        mag.magnetic_field.y = my*1.0
        mag.magnetic_field.z = mz*1.0

        # Publish the message
        self.imu_publisher.publish(imu)
        self.mag_publisher.publish(mag)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass

    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()