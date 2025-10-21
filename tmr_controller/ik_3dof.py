import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class IK3DOF(Node):
    def __init__(self):
        super().__init__('ik_3dof')

        self.link_lengths = [0.075, 0.39, 0.55]  

        self.create_subscription( Point, '/arm_pos', self.target_position_callback, 10)

        self.pub_q1 = self.create_publisher(Float64, '/arm_teleop/joint1', 1)
        self.pub_q2 = self.create_publisher(Float64, '/arm_teleop/joint2', 1)
        self.pub_q3 = self.create_publisher(Float64, '/arm_teleop/joint3_2', 1)

        self.x_des = None
        self.y_des = None
        self.z_des = None

        self.l1, self.l2, self.l3 = self.link_lengths

    def target_position_callback(self, msg):
        self.x_des = msg.x
        self.y_des = msg.y
        self.z_des = msg.z

        # Call the inverse kinematics function
        theta1, theta2, theta3 = self.calculate_inverse_kinematics()

        # Publish the joint angles
        q1_msg = Float64()
        q1_msg.data = theta1
        self.pub_q1.publish(q1_msg)

        q2_msg = Float64()
        q2_msg.data = theta2
        self.pub_q2.publish(q2_msg)

        q3_msg = Float64()
        q3_msg.data = theta3
        self.pub_q3.publish(q3_msg)

    def calculate_inverse_kinematics(self):

        if self.x_des is None or self.y_des is None or self.z_des is None:
            self.get_logger().warn('Target position not set yet.')
            return None, None, None

        theta1 = np.arctan2(self.y_des, self.x_des)
        r = np.sqrt(self.x_des**2 + self.y_des**2)
        s = (r**2 + (self.z_des - self.l1)**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        theta3 = np.arctan2(-np.sqrt(1 - s**2), s)
        theta2 = np.arctan2(self.z_des - self.l1, r) - np.arctan2(self.l3 * np.sin(theta3), self.l2 + self.l3 * np.cos(theta3))

        # Convert angles from radians to degrees
        theta1 = np.degrees(theta1)
        theta2 = np.degrees(theta2)
        theta3 = np.degrees(theta3)
        # Normalize angles to be within [-180, 180]
        theta1 = (theta1 + 180) % 360 - 180
        theta2 = (theta2 + 180) % 360 - 180
        theta3 = (theta3 + 180) % 360 - 180
        return theta1, theta2, theta3
    
def main(args=None):
    rclpy.init(args=args)
    ik_3dof = IK3DOF()
    rclpy.spin(ik_3dof)
    ik_3dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
