import rclpy
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
        self.pub_save_map = self.create_publisher(Bool, "/save_map", 1)  #guarda punto en el mapa
        #subs
        self.sub_doneArm = self.create_subscription(Bool, "/done_arm", self.callback_doneArm, 10)#recibe si ya termino la rutina de recojer rocas con el brazo
        self.sub_visCenter = self.create_subscription(Bool, "/object_centered", self.callback_visCenter, 10)#recibe si ya el robot esta centrado
        self.sub_visDetect=self.create_subscription(Bool, "/object_detected", self.callback_visDetect, 10)#recibe si ya el robot detecto la roca

        self.create_timer(0.1, self.master_loop)

        self.done_arm= False
        self.vis_center= False
        self.vis_detect= False

        self.phase = "PATH"      
        self.last_pub = {"center":False, "arm":False, "path":True, "map":False}

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
        # ---------- LÓGICA DE TRANSICIÓN ----------
        if self.phase == "PATH":
            if self.vis_detect and not self.vis_center:
                self.phase = "CENTER"

        elif self.phase == "CENTER":
            if self.vis_center:                 # ya quedó centrado
                self.phase = "ARM"              # pasa a brazo
            elif not self.vis_detect:
                self.phase = "PATH"

        elif self.phase == "ARM":
            if self.done_arm:                   # brazo terminó
                self.phase = "PATH"

                self.done_arm = self.vis_center = self.vis_detect = False

        # ---------- SALIDAS DEPENDEN DE LA FASE ----------
        desired = {
            "center": self.phase == "CENTER",
            "arm":    self.phase == "ARM",
            "path":   self.phase == "PATH",
            "map":    self.phase == "ARM"       # guardar mapa solo mientras está el brazo
        }

        # Publica solo si cambia (evita spam)
        if desired != self.last_pub:
            self.pub_initVision.publish(Bool(data=desired["center"]))
            self.pub_initArm.publish   (Bool(data=desired["arm"]))
            self.pub_path_init.publish (Bool(data=desired["path"]))
            self.pub_save_map.publish  (Bool(data=desired["map"]))
            self.last_pub = desired

def main(args=None):
    rclpy.init(args=args)
    node = ControllerMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    