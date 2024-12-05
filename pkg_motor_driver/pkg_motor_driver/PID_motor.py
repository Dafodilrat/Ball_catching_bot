import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)

    Parameters:
    - x, y, z, w: float
        Components of the quaternion.

    Returns:
    - roll: float
        Rotation around the X-axis in radians.
    - pitch: float
        Rotation around the Y-axis in radians.
    - yaw: float
        Rotation around the Z-axis in radians.
    """
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, actual, dt):
        error = setpoint - actual
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to the motor's supported range
        return max(min(output, self.max_output), -self.max_output)


class MecanumRobot(Node):

    def __init__(self):
        super().__init__('robot_pid')

        # PID Controllers for x, y, and orientation
        self.pid_x = PIDController(kp=1.0, ki=0.0, kd=0.5, max_output=10.0)
        self.pid_y = PIDController(kp=1.0, ki=0.0, kd=0.5, max_output=10.0)
        self.pid_theta = PIDController(kp=1.0, ki=0.0, kd=0.5, max_output=10.0)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_position = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.target_position = [0.0, 0.0, 0.0]  # [x_target, y_target, theta_target]
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.dt = 0.1  # Time delta for PID control

    def odom_callback(self, data):
        self.current_position[0] = data.pose.pose.position.x
        self.current_position[1] = data.pose.pose.position.y
        print("current location :({},{})".format(data.pose.pose.position.x,data.pose.pose.position.y))

        orientation_q = data.pose.pose.orientation
        euler_angles = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
        self.current_position[2] = euler_angles[2]  # Yaw angle (theta)

    def move_to_target(self, target_x, target_y, target_theta=0.0):
        self.target_position = [target_x, target_y, target_theta]
        self.get_logger().info(f"Target set to: {self.target_position}")

    def control_loop(self):
        # PID for x, y, and theta
        vel_x = self.pid_x.compute(self.target_position[0], self.current_position[0], self.dt)
        vel_y = self.pid_y.compute(self.target_position[1], self.current_position[1], self.dt)
        vel_theta = self.pid_theta.compute(self.target_position[2], self.current_position[2], self.dt)

        # Publish velocity commands (ensuring they are in the range [-10, 10])
        cmd = Twist()
        cmd.linear.x = max(min(vel_x, 10), -10)
        cmd.linear.y = max(min(vel_y, 10), -10)
        cmd.angular.z = max(min(vel_theta, 10), -10)
        self.pub.publish(cmd)

        # Check if the robot has reached the target
        if (abs(self.target_position[0] - self.current_position[0]) < 0.05 and
                abs(self.target_position[1] - self.current_position[1]) < 0.05 and
                abs(self.target_position[2] - self.current_position[2]) < 0.05):
            self.get_logger().info("Target reached!")
            cmd = Twist()  # Stop the robot
            self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    robot = MecanumRobot()

    target_x = float(input("Enter target X coordinate: "))
    target_y = float(input("Enter target Y coordinate: "))
    robot.move_to_target(target_x, target_y)

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
