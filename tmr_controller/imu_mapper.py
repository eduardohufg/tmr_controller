import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool
import cv2
import numpy as np

class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')
        
        # Mapa (800x1000 píxeles, fondo blanco)
        self.map = np.ones((800, 1000, 3), dtype=np.uint8) * 255
        
        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.color_sub = self.create_subscription(Int16, '/object_color', self.color_callback, 10)
        self.object_centered_sub = self.create_subscription(Bool, '/object_centered', self.fixed_point_callback, 10)
        
        # Variables de estado
        self.current_color = (0, 0, 255)  # Rojo por defecto para puntos fijos
        self.robot_color = (255, 255, 0)     # Verde para el robot
        self.robot_pos = (0, 0)            # Posición actual del robot
        self.fixed_points = []              # Lista de puntos fijos [(x1, y1), (x2, y2), ...]
        
        # Diccionario de colores para /color_code
        self.color_dict = {
            0: (0, 0, 0),      # Negro
            1: (0, 0, 255),    # Rojo
            2: (0, 255, 0),    # Verde (usado para el robot)
            3: (255, 0, 0),    # Azul
            4: (255, 255, 0),  # Amarillo
        }
        
        self.get_logger().info("Mapper node ready. Waiting for data...")
    
    def color_callback(self, msg):
        # Actualizar color para puntos fijos
        self.current_color = self.color_dict.get(msg.data, (0, 0, 255))  # Rojo por defecto
    
    def fixed_point_callback(self, msg):
        # Añadir punto fijo si el booleano es True
        if msg.data:
            x, y = self.robot_pos
            self.fixed_points.append((x, y))
            self.get_logger().info(f"Fixed point added at: ({x:.2f}, {y:.2f})")
    
    def odom_callback(self, msg):
        # Actualizar posición del robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pos = (x, y)
        
        # Convertir a píxeles
        x_px = int(x * 100)
        y_px = int(y * 100)
        
        # Limpiar el mapa (fondo blanco)
        self.map = np.ones((800, 1000, 3), dtype=np.uint8) * 255
        
        # Dibujar todos los puntos fijos acumulados
        for (fx, fy) in self.fixed_points:
            fx_px = int(fx * 100)
            fy_px = 800-int(fy * 100)
            cv2.circle(self.map, (fx_px, fy_px), 5, self.current_color, -1)
        
        # Dibujar el robot (verde y más grande)
        cv2.circle(self.map, (x_px, y_px), 8, self.robot_color, -1)
        
        # Mostrar mapa
        cv2.imshow("2D Map - Robot (Green) | Fixed Points (Color)", self.map)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    rclpy.spin(node)
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
