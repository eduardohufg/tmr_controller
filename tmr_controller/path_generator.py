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
        [0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [8.0, 0.0], [10.0, 0.0],
        [10.0, 1.0], [8.0, 1.0], [6.0, 1.0], [4.0, 1.0], [2.0, 1.0], [0.0, 1.0],
        [0.0, 2.0], [2.0, 2.0], [4.0, 2.0], [6.0, 2.0], [8.0, 2.0], [10.0, 2.0],
        [10.0, 3.0], [8.0, 3.0], [6.0, 3.0], [4.0, 3.0], [2.0, 3.0], [0.0, 3.0],
        [0.0, 4.0], [2.0, 4.0], [4.0, 4.0], [6.0, 4.0], [8.0, 4.0], [10.0, 4.0],
        [10.0, 5.0], [8.0, 5.0], [6.0, 5.0], [4.0, 5.0], [2.0, 5.0], [0.0, 5.0],
        [0.0, 6.0], [2.0, 6.0], [4.0, 6.0], [6.0, 6.0], [8.0, 6.0], [10.0, 6.0],
        [10.0, 7.0], [8.0, 7.0], [6.0, 7.0], [4.0, 7.0], [2.0, 7.0], [0.0, 7.0],
        [0.0, 8.0], [2.0, 8.0], [4.0, 8.0], [6.0, 8.0], [8.0, 8.0], [10.0, 8.0],
        [10.0, 9.0], [8.0, 9.0], [6.0, 9.0], [4.0, 9.0], [2.0, 9.0], [0.0, 9.0]
                            ]
        self.msg = Point() 
        self.msg_finished = Bool()
        self.arrived = False
        self.t0 = time.time()
        
    
    def callback_arrived(self, msg):
        if msg.data:
            self.get_logger().info("Arrived at the point")
            self.arrived = True
            self.point_list.pop(0)
        else:
            self.arrived = False

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