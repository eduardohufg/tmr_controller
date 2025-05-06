import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
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
        self.pub_gripper = self.create_publisher(Float64, "/arm_teleop/gripper", 1)

        # Initialize variables

        self.arm_pos = Point()

        self.init_arm = False

        self.arm_pos.x = 0.0
        self.arm_pos.y = 0.0
        self.arm_pos.z = 0.0

        self.gripper = Float64()

        self.create_timer(0.1, self.arm_routine_loop)

    def callback_initArm(self, msg):

        self.init_arm = msg.data
    
    def arm_routine_loop(self):



        if self.init_arm:
            self.pub_doneArm.publish(Bool(data=False))
            self.get_logger().info("Arm Routine Started")
            self.arm_pos.x = 0.35
            self.arm_pos.y = 0.0
            self.arm_pos.z = -0.45
            
            self.gripper.data = -1.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)

            time.sleep(5)
            self.arm_pos.x = 0.35
            self.arm_pos.y = 0.0
            self.arm_pos.z = -0.45
            
            self.gripper.data = 1.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)



            time.sleep(5)

            self.arm_pos.x = 0.32
            self.arm_pos.y = 0.0
            self.arm_pos.z = -0.1
            self.gripper.data = 0.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)


            time.sleep(5)


            self.arm_pos.x = 0.3
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.2
            self.gripper.data = 0.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)



            time.sleep(4)
            

            self.arm_pos.x = 0.1
            self.arm_pos.y = -0.15
            self.arm_pos.z = 0.2
            self.gripper.data = 0.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)




            time.sleep(10)

            self.arm_pos.x = 0.1
            self.arm_pos.y = -0.15
            self.arm_pos.z = 0.2
            self.gripper.data = -1.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)
            time.sleep(2)

            self.arm_pos.x = 0.1
            self.arm_pos.y = 0.0
            self.arm_pos.z = 0.3
            self.gripper.data = 1.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)

            time.sleep(3)

            self.pub_doneArm.publish(Bool(data=True))
            self.get_logger().info("Arm Routine Finished")
            self.init_arm = False

        else:
            self.arm_pos.x = 0.32
            self.arm_pos.y = 0.0
            self.arm_pos.z = -0.1
            self.gripper.data = 0.0
            self.pub_armPos.publish(self.arm_pos)
            self.pub_gripper.publish(self.gripper)
            

        

def main(args=None):
    rclpy.init(args=args)
    node = ArmRoutine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()