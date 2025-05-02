import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float64
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import tf_transformations
import sys
from typing import Optional
from enum import Enum, auto


# class TrafficFSM:
#     CENTER_ROUTINE  = "center_routine"
#     ARM_ROUTINE = "arm_routine" 
#     FOLLOW_PATH = "follow_path"



class TrafficFSM(Enum):
    FOLLOW_PATH     = auto()
    CENTER_ROUTINE  = auto()
    ARM_ROUTINE     = auto()


class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Main Controller node has started")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)


        self.sub_points = self.create_subscription(Point, "/next_point", self.callback_points, 10)
        self.sub_color_g = self.create_subscription(Bool, '/init_center', self.callback_init_center,10)
        self.sub_color_r = self.create_subscription(Bool, '/init_arm', self.callback_init_arm,10)
        self.sub_color_y = self.create_subscription(Bool, '/init_path', self.callback_init_path,10)
        self.create_subscription(Float64, 'angle', self.callback_angle, 10)
        self.create_subscription(Point, '/object_offset', self.callback_offsert, 10) 
        self.create_subscription(Odometry, "/odom", self.callback_odom, 1)

        self.create_timer(0.01, self.control_loop)

        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.init_center_state = False
        self.init_arm_state = False
        self.init_path_state = False

        self.kv = 0.2
        self.kw = 1.0

        self.off_x, self.off_y = 0.0, 0.0

        self.state = TrafficFSM.FOLLOW_PATH

        self.arrived = Bool()

    def callback_init_center(self, msg):
        self.init_center_state = msg.data

    def callback_init_arm(self, msg):
        self.init_arm_state = msg.data

    def callback_init_path(self, msg):
        self.init_path_state = msg.data


    def callback_offsert(self, msg):
        self.off_x = msg.x
        self.off_y = msg.y


    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def callback_angle(self, msg):

        self.theta = msg.data


    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        self.get_logger().info(f"Next point: {self.x_d}, {self.y_d}")

    # def control_loop(self):

    #     if self.x is not None and self.y is not None and self.x_d is not None and self.y_d is not None:

    #         signal = self.current_signal()

    #         self.get_logger().info(f"Current signal: {signal}")

    #         if self.state == TrafficFSM.ARM_ROUTINE:
    #             if signal == "follow_path":
    #                 self.state = TrafficFSM.FOLLOW_PATH

    #         elif self.state == TrafficFSM.CENTER_ROUTINE:
    #             if signal == "arm_routine":
    #                 self.state = TrafficFSM.ARM_ROUTINE

    #         elif self.state == TrafficFSM.FOLLOW_PATH:
    #             if signal == "center_routine":
    #                 self.state = TrafficFSM.CENTER_ROUTINE
    #             elif signal == "arm_routine":
    #                 self.state = TrafficFSM.ARM_ROUTINE
            
    #         else:
    #             if signal == "follow_path":
    #                 self.state = TrafficFSM.FOLLOW_PATH
    #             elif signal == "arm_routine":
    #                 self.state = TrafficFSM.ARM_ROUTINE
    #             elif signal == "center_routine":
    #                 self.state = TrafficFSM.CENTER_ROUTINE

    #         if self.state == TrafficFSM.FOLLOW_PATH:
    #             self.advance(kv=0.2, kw=1.0)
    #         elif self.state == TrafficFSM.CENTER_ROUTINE:
    #             self.go_to_center()
    #         elif self.state == TrafficFSM.ARM_ROUTINE:
    #             self.stop_velocity()



    def control_loop(self):
        if None in (self.x, self.y, self.x_d, self.y_d):
            return

        event = self.current_signal()
        if event is not None:
            self.state = event

        if self.state is TrafficFSM.FOLLOW_PATH:
            self.advance(kv=0.2, kw=1.0)

        elif self.state is TrafficFSM.CENTER_ROUTINE:
            self.go_to_center()

        elif self.state is TrafficFSM.ARM_ROUTINE:
            self.stop_velocity()


    # def current_signal(self):
    #     if self.init_path_state:
    #         return "follow_path"
    #     elif self.init_center_state:
    #         return "center_routine"
    #     elif self.init_arm_state:
    #         return "arm_routine"
    #     else:
    #         return None
        
    def current_signal(self) -> TrafficFSM | None:
  
        if self.init_center_state:
            self.init_center_state = False
            return TrafficFSM.CENTER_ROUTINE

        if self.init_arm_state:
            self.init_arm_state = False
            return TrafficFSM.ARM_ROUTINE

        if self.init_path_state:       # ← ojo al typo, cámbialo donde lo declares
            self.init_path_state = False
            return TrafficFSM.FOLLOW_PATH

        return None

    def go_to_center(self):
        msg = Twist()
        max_ang_speed = 1.5
        max_lin_speed = 0.5

        self.get_logger().info(f"centering: {self.off_x}, {self.off_y}")
        
        if abs(self.off_x) > 20:
            msg.linear.x = 0.0
            msg.angular.z = max(-max_ang_speed, min(max_ang_speed, self.kw * self.off_x))

        elif abs(self.off_y) > 20:
            
            msg.linear.x = max(-max_lin_speed, min(max_lin_speed, self.kv * self.off_y))
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)
    
    def stop_velocity(self):

        self.get_logger().info("Stopping robot init arm routine")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
            
    def advance(self, kv=0.2, kw=1.0):

       
        msg = Twist()

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx**2 + Dy**2)

        angle_to_goal = math.atan2(Dy, Dx)
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        self.get_logger().info(f"Distance to target: {distance}")

        if distance > 0.1:
            if abs(angle_error) > 0.1:  # Fase de rotación
                msg.linear.x = 0.0
                msg.angular.z = kw * angle_error
            else:  # Fase de avance
                msg.linear.x = kv
                msg.angular.z = kw * angle_error

            self.arrived.data = False
            self.pub_arrived.publish(self.arrived)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)

        self.pub.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    