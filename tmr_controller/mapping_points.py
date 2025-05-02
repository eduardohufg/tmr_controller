import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16
import cv2
import numpy as np

class MapDrawer(Node):
    def __init__(self):
        super().__init__('map_drawer')
        # Parámetros del mapa en metros y pixeles
        self.width_m = 10.0
        self.height_m = 8.0
        self.width_px = 1000
        self.height_px = 800
        self.scale_x = self.width_px  / self.width_m   # 100 px/m
        self.scale_y = self.height_px / self.height_m  # 100 px/m

        self.x = 0.0
        self.y = 0.0
        self.current_color = None

        self.image = np.ones((self.height_px, self.width_px, 3), dtype=np.uint8) * 255

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.create_subscription(Bool,      '/save_map', self.draw_callback, 10)
        self.create_subscription(Int16,    'object_color', self.color_callback, 10)

        # Timer para refrescar ventana
        self.create_timer(0.1, self.timer_callback)

        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('map', 600, 480)
        self.get_logger().info('MapDrawer iniciado')

    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def color_callback(self, msg):
        # Actualizar color ('red','green','blue',...)
        self.current_color = msg.data

    def draw_callback(self, msg: Bool):
        if msg.data and self.current_color is not None:
            x = self.x
            y = self.y
            # Convertir metros a pixeles (origen en esquina inferior izquierda)
            px = int(x * self.scale_x)
            py = self.height_px - int(y * self.scale_y)
            # Definir color BGR
            colors = {
                1:   (0,0,255),
                2: (0,255,0),
                3:  (255,0,0),
            }
            bgr = colors.get(self.current_color, (0,0,0))
            # Dibujar círculo
            cv2.circle(self.image, (px, py), 5, bgr, -1)
            self.get_logger().info(f'Dibujado círculo {self.current_color} en ({x:.2f} m, {y:.2f} m)')

    def timer_callback(self):
        # Mostrar mapa
        cv2.imshow('map', self.image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MapDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
