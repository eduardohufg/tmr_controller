import rclpy
from std_msgs.msg import Float64
from adafruit_servokit import ServoKit
import time
from rclpy.node import Node

def my_map(value: float, in_min: float, in_max,out_min: float, out_max: float) -> float:

    targetPos: float = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return targetPos

class servos_main(Node):

    def __init__(self):
    
        super().__init__('servos_arm_tmr')
        print("Enter main")
        self.kit = ServoKit(channels=16)

        self.servo_rotacion = 7
        
        self.kit.servo[self.servo_rotacion].set_pulse_width_range(500,2500)
        self.kit.servo[self.servo_rotacion].actuation_range=360

        self.kit.servo[self.servo_rotacion].angle=180
        
        self.subscriber_servo_rotacion = self.create_subscription(Float64,"/arm_teleop/joint2_s", self.callback3,10)
        self.subscriber_servo_rotacion
        
    def limits(self,data, liminf,limsup):

        if data<liminf:
            data=liminf
        elif data>limsup:
            data=limsup
        return data
        
            
            
    def callback3(self,data):
        try:
            ndata=my_map(data.data,0,0,360,0,-180,0,180,0)

            ndata=self.limits(ndata,-180,180)

            print("ndata: ",ndata)

            self.kit.servo[self.servo_rotacion].angle = int(ndata)
        
        except Exception as e:
            print(data.data)
            print(e)
            
            
def main(args=None):
    rclpy.init(args=args)
    listener=servos_main()
    # spin es un nodo de ejecución que escucha a otro
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
