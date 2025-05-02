import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Bool

class ControllerMaster(Node):
    def __init__(self):
        super().__init__("controller_master")
        self.get_logger().info("Controller Master Node Started")
        #pubs
        self.pub_initVision = self.create_publisher(Bool, "/init_center", 1)#inicia la rutina de center mediante vision
        self.pub_initArm = self.create_publisher(Bool, "/init_arm", 1)#inicia la rutina de recojer rocas con el brazo
        self.pub_path_init = self.create_publisher(Bool, "/init_path", 1)#inicia la rutina de path
        self.pub_save_map = self.create_publisher(Bool, "save_map", 1)  #guarda punto en el mapa
        #subs
        self.sub_doneArm = self.create_subscription(Bool, "/doneArm", self.callback_doneArm, 10)#recibe si ya termino la rutina de recojer rocas con el brazo
        self.sub_visCenter = self.create_subscription(Bool, "/object_centered", self.callback_visCenter, 10)#recibe si ya el robot esta centrado
        self.sub_visDetect=self.create_subscription(Bool, "/object_detected", self.callback_visDetect, 10)#recibe si ya el robot detecto la roca

        self.create_timer(0.1, self.master_loop)

        self.done_arm= False
        self.vis_center= False
        self.vis_detect= False

        self.collect_msg=Bool()
        self.do_path_msg=Bool()
        self.do_arm_msg =Bool()
        self.save_map_msg=Bool()

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
        if self.vis_detect and not self.vis_center:
            state = "CENTER"
        elif self.vis_center and self.vis_detect and not self.done_arm:
            state = "ARM"
        else:
            state = "PATH"

        # Mapear estado → salidas
        self.collect_msg.data  = (state == "CENTER")
        self.do_arm_msg.data   = (state == "ARM")
        self.do_path_msg.data  = (state == "PATH")
        self.save_map_msg.data = (state == "ARM")

        # Publicar
        self.pub_initVision.publish(self.collect_msg)
        self.pub_initArm.publish(self.do_arm_msg)
        self.pub_path_init.publish(self.do_path_msg)
        self.pub_save_map.publish(self.save_map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    