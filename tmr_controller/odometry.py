#!/usr/bin/env python3
import rclpy, math, time, tf_transformations
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy import qos

class OdometryClass(Node):
    def __init__(self):
        super().__init__("odometry")
        self.get_logger().info("Robot pose estimation by odometry node.")
        self.create_timer(0.01, self.odometry_callback)
        self.pub = self.create_publisher(Odometry, 'odom', 1)
        self.create_subscription(Float32, 'VelocityEncR', self.wR_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL', self.wL_cb, qos.qos_profile_sensor_data)
        
        # Initialize variables
        self.wR = 0.0 
        self.wL = 0.0 
        self.x = 0.0 
        self.y = 0.0 
        self.q = 0.0 
        self.t0 = time.time() 
        self.r = 0.0505  
        self.L = 0.1725 
        
        # Initialize the odometry message
        self.odom_msg = Odometry()

    def wR_cb(self, msg):
        self.wR = msg.data
    
    def wL_cb(self, msg):
        self.wL = msg.data

    def odometry_callback(self):
        elapsed_time = time.time() - self.t0 
        self.t0 = time.time()  
        v = (self.wR + self.wL)*self.r / 2.0
        w = (self.wR - self.wL)*self.r / self.L


        if abs(w) > 1e-6:
            dx = (v / w) * (math.sin(self.q + w * elapsed_time) - math.sin(self.q))
            dy = (v / w) * (-math.cos(self.q + w * elapsed_time) + math.cos(self.q))
        else:
            dx = v * math.cos(self.q) * elapsed_time
            dy = v * math.sin(self.q) * elapsed_time

        self.x += dx
        self.y += dy
        self.q += w * elapsed_time

        # Create odometry message
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.q)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    nodeh = OdometryClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()