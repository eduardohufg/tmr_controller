import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import tf_transformations

class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Main Controller node has started")

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
        self.pub_donevision = self.create_publisher(Bool, "/Donevision", 1)

        # Subscriptions
        self.sub_points = self.create_subscription(Point, "/next_point", self.callback_points, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.sub_initvision = self.create_subscription(Bool, "/initVision", self.callback_vision, 10)
        self.sub_path_init = self.create_subscription(Bool, "/Path_init", self.callback_path, 10)
        self.sub_offset = self.create_subscription(Point, "/object_offset", self.callback_offset, 10)
        self.pub_initArm = self.create_subscription(Bool, "/initArm", self.callback_arm, 10) 

        self.create_timer(0.01, self.control_loop)

        self.x = None  
        self.y = None  
        self.theta = None
        self.x_d = None 
        self.y_d = None 
        self.offset = Point() 
        self.in_path_mode = False 
        self.object_detected = False 
        self.state = "IDLE"  # states: IDLE, NAVIGATING, OBJECT_DETECTED, CENTERING

        self.arrived_msg = Bool()

    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.theta = tf_transformations.euler_from_quaternion(orientation_list)[2]

    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y
        self.get_logger().info(f"Next point received: {self.x_d}, {self.y_d}")
        self.state = "NAVIGATING"

    def callback_vision(self, msg):
        self.object_detected = msg.data
        if self.object_detected:
            self.get_logger().info("Object detected, Stopping navigation.")
            self.state = "OBJECT_DETECTED"

    def callback_path(self, msg):
        self.in_path_mode = msg.data
        self.get_logger().info(f"Path mode: {'Enabled' if self.in_path_mode else 'Disabled'}")

    def callback_offset(self, msg):
        self.offset = msg
        self.get_logger().info(f"Offset received: {self.offset.x}, {self.offset.y}")

    def callback_arm(self, msg):
        if msg.data: 
            self.get_logger().info("Arm routine startd. Centering the rover.")
            self.state = "CENTERING" 

    def control_loop(self):
        if self.state == "IDLE":
            return

        if self.state == "OBJECT_DETECTED":
            self.stop_rover()
            return

        if self.state == "NAVIGATING" and self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:
            self.navigate_to_point()
            return

        if self.state == "CENTERING":
            self.center_rover()
            return

    def navigate_to_point(self):
        kv = 0.2  
        kw = 1.0  
        dx = self.x_d - self.x
        dy = self.y_d - self.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd_msg = Twist()
        if distance > 0.1: 
            if abs(angle_error) > 0.08:  # Fase de rotación
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = kw * angle_error
            else:  # fase de avance
                cmd_msg.linear.x = kv
                cmd_msg.angular.z = kw * angle_error

            self.arrived_msg.data = False
        else:  
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.arrived_msg.data = True
            self.state = "IDLE" 

        self.pub_cmd_vel.publish(cmd_msg)
        self.pub_arrived.publish(self.arrived_msg)

    def center_rover(self):
        return #por definir 


    def stop_rover(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd_msg)
        self.get_logger().info("Rover stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()