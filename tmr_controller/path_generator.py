import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from std_msgs.msg import Bool
import sys

class PathGeneratorNode(Node):
    def __init__ (self):
        super().__init__("path_generator")
        self.get_logger().info("Path Generator node has started")

        self.pub = self.create_publisher(Point, "/next_point", 1)
        self.pub_finished = self.create_publisher(Bool, "/path_finished", 1)
        self.sub_arrived = self.create_subscription(Bool, "/arrived", self.callback_arrived, 10)

        self.create_timer(0.01, self.generate_path)                    
        
        
        self.point_list = [
        [0.0, 0.0], [8.5, 0.0],
        [8.5, 1.5],[0.0, 1.5],
        [0.0, 3.0], [8.5, 3.0],
        [8.5, 4.5],[0.0, 4.5],
        [0.0, 6.0], [8.5, 6.0],
        [8.5, 7.5],[0.0, 7.5],
        [0.0, 8.2],
        [0.0, 1.0]]

        self.arrived_prev = False
                           
        self.msg = Point() 
        self.msg_finished = Bool()
        self.arrived = False
        self.t0 = time.time()
        
    
    def callback_arrived(self, msg):
        # Solo actúa en el momento en que cambia de False → True
        if msg.data and not self.arrived_prev:
            self.get_logger().info("Arrived at the point")
            if self.point_list:        # seguridad extra
                self.point_list.pop(0)
        # memoriza el estado para el siguiente mensaje
        self.arrived_prev = msg.data

    def generate_path(self):
        if (len(self.point_list) > 0):
            [x, y] = self.point_list[0]
            self.msg.x = x
            self.msg.y = y

            self.pub.publish(self.msg)

        else:
            self.get_logger().info("Path generation finished")
            self.msg_finished.data = True
            self.pub_finished.publish(self.msg_finished)
            self.destroy_node()
            return

        
def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit()

if __name__ == "__main__":
    main()
