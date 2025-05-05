import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import time

class ArmRoutine(Node):
    def __init__(self):
        super().__init__("arm_routine")
        self.get_logger().info("Arm Routine Node Started")
        #suds
        self.create_subscription(Bool, "/init_arm", self.callback_initArm, 10)#inicia la rutina de recojer rocas con el brazo

        #pubs
        self.pub_doneArm = self.create_publisher(Bool, "/done_arm", 1)
        self.pub_armPos = self.create_publisher(Point, "/arm_pos", 1)

        # Initialize variables

        self.arm_pos = Point()

        self.init_arm = False

        self.arm_pos.x = 0.0
        self.arm_pos.y = 0.0
        self.arm_pos.z = 0.0

        self.create_timer(0.1, self.arm_routine_loop)

    def callback_initArm(self, msg):

        self.init_arm = msg.data
    
    def arm_routine_loop(self):

        if self.init_arm:
            self.pub_doneArm.publish(Bool(data=False))
            self.get_logger().info("Arm Routine Started")
            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.0

            self.pub_armPos.publish(self.arm_pos)

            time.sleep(2)
            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 1.0
            self.pub_armPos.publish(self.arm_pos)
            time.sleep(2)

            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.0
            self.pub_armPos.publish(self.arm_pos)
            time.sleep(2)
            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.0
            self.pub_armPos.publish(self.arm_pos)
            time.sleep(2)
            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.0
            self.pub_armPos.publish(self.arm_pos)
            time.sleep(2)
            self.arm_pos.x = 0.0
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.0
            self.pub_armPos.publish(self.arm_pos)
            time.sleep(2)

            self.pub_doneArm.publish(Bool(data=True))
            self.get_logger().info("Arm Routine Finished")
            self.init_arm = False

        

def main(args=None):
    rclpy.init(args=args)
    node = ArmRoutine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()