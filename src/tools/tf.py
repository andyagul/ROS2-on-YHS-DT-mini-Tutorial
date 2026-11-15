import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion

class QuaternionConverter(Node):

    def __init__(self):
        super().__init__('quaternion_converter')
        self.publisher_ = self.create_publisher(Quaternion, 'quaternion_topic', 10)

    def convert_angles_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.w = cy * cp * cr + sy * sp * sr
        quaternion.x = cy * cp * sr - sy * sp * cr
        quaternion.y = sy * cp * sr + cy * sp * cr
        quaternion.z = sy * cp * cr - cy * sp * sr

        return quaternion

    def publish_quaternion(self, quaternion):
        self.publisher_.publish(quaternion)
        self.get_logger().info('Quaternion published: {}'.format(quaternion))

def main(args=None):
    rclpy.init(args=args)
    converter = QuaternionConverter()

    roll = float(input("Enter roll angle (in degrees): "))
    pitch = float(input("Enter pitch angle (in degrees): "))
    yaw = float(input("Enter yaw angle (in degrees): "))

    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    quaternion = converter.convert_angles_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    converter.publish_quaternion(quaternion)

    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
