import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Bool

class ControllerMaster(Node):
    def __init__(self):
        super().__init__("controller_master")
        self.get_logger().info("Controller Master Node Started")
        #pubs
        self.pub_initVision = self.create_publisher(Bool, "/initVision", 1)#inicia la rutina de center mediante vision
        self.pub_initArm = self.create_publisher(Bool, "/initArm", 1)#inicia la rutina de recojer rocas con el brazo
        self.pub_path_init = self.create_publisher(Bool, "/Path_init", 1)#inicia la rutina de path
        self.pub_save_map = self.create_publisher(Bool, "SaveMap", 1)  #guarda punto en el mapa
        #subs
        self.sub_doneArm = self.create_subscription(Bool, "/doneArm", self.callback_doneArm, 10)#recibe si ya termino la rutina de recojer rocas con el brazo
        self.sub_visCenter = self.create_subscription(Bool, "/object_centered", self.callback_visCenter, 10)#recibe si ya el robot esta centrado
        self.sub_visDetect=self.create_subscription(Bool, "/object_detected", self.callback_visDetect, 10)#recibe si ya el robot detecto la roca

        self.create_timer(0.01, self.master_loop)

        self.done_arm=Bool()
        self.vis_center=Bool()
        self.vis_detect=Bool()

        self.collect=Bool()
        self.doPath=Bool()
        self.doArm =Bool()
        self.saveMap=Bool()

    def callback_doneArm(self,msg):
        self.done_arm=msg.data
        if self.done_arm==True:
            self.get_logger().info("Done Arm")

    
    def callback_visDetect(self,msg):
        self.vis_detect=msg.data
        if self.vis_detect==True:
            self.get_logger().info("Detect Rock")
    
    def callback_visCenter(self,msg):
        self.vis_center=msg.data
        if self.vis_center==True:
            self.get_logger().info("Center Rock")

    def master_loop(self):

        if self.vis_detect==True and self.vis_center==False:
            self.get_logger().info("Center Rock")
            self.collect.data=True
            self.doPath.data=False
            self.doArm.data=False
            self.saveMap.data=False

        elif self.vis_center==True and self.vis_detect==True and self.done_arm==False:
            self.get_logger().info("Init Arm")
            self.collect.data=False
            self.doPath.data=False
            self.doArm.data=True
            self.saveMap.data=True
            
        else:
            self.get_logger().info("Init Path")
            self.collect.data=False
            self.doPath.data=True
            self.doArm.data=False
            self.saveMap.data=False

        # Publish messages
        self.pub_initVision.publish(self.collect)
        self.pub_path_init.publish(self.doPath)
        self.pub_initArm.publish(self.doArm)
        self.pub_save_map.publish(self.saveMap) 

def main(args=None):
    rclpy.init(args=args)
    node = ControllerMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    