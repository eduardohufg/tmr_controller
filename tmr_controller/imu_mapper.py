import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3  # Para roll, pitch, yaw
from nav_msgs.msg import Odometry
import pygame
import math
import numpy as np
from pygame.locals import * 

class IMUMapper(Node):
    def __init__(self):
        super().__init__('imu_2d_mapper')

        # Suscriptores
        self.create_subscription(Vector3, "/imu/euler_angles", self.imu_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Configuración Pygame
        pygame.init()
        self.width, self.height = 1000, 900  # 10x9 metros (1px = 1cm)
        self.cell_size = 5
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Mapa 2D")

        # Variables de estado
        self.terrain_map = np.zeros((self.height, self.width, 3))  # Mapa RGB
        self.robot_pos = [self.width // 2, self.height // 2]  # Posición inicial
        self.robot_angle = 0.0  # Orientación (yaw) en radianes
        self.meters_to_pixels = 100  # 100px = 1 metro
        self.max_angle = math.radians(30)  # Límite de ±30°

        # Paleta de colores
        self.flat_color = (0, 255, 0)       # Verde
        self.slight_color = (255, 255, 0)   # Amarillo
        self.moderate_color = (255, 165, 0) # Naranja
        self.steep_color = (255, 0, 0)      # Rojo

    def get_inclination_color(self, pitch, roll):
        """Escala de colores para inclinación"""
        inclination = math.sqrt(pitch**2 + roll**2)
        inclination_deg = abs(math.degrees(inclination))

        if inclination_deg < 5: return self.flat_color
        elif inclination_deg < 15: return self.slight_color
        elif inclination_deg < 30: return self.moderate_color
        else: return self.steep_color

    def odom_callback(self, msg):
        """Actualiza SOLAMENTE la posición desde odometría"""
        # Convertir metros a píxeles
        self.robot_pos[0] = int(msg.pose.pose.position.x * self.meters_to_pixels + self.width/2)
        self.robot_pos[1] = int(-msg.pose.pose.position.y * self.meters_to_pixels + self.height/2)

    def imu_callback(self, msg):
        """Procesa roll, pitch (de Vector3) y actualiza el mapa"""
        try:
            roll = max(min(msg.x, self.max_angle), -self.max_angle)
            pitch = max(min(msg.y, self.max_angle), -self.max_angle)
            self.robot_angle = msg.z  # Usamos el yaw directamente del mensaje

            # Actualizar mapa
            if 0 <= self.robot_pos[0] < self.width and 0 <= self.robot_pos[1] < self.height:
                self.terrain_map[self.robot_pos[1], self.robot_pos[0]] = self.get_inclination_color(pitch, roll)

            self.get_logger().info(
                f"Pos: {self.robot_pos} | Yaw: {math.degrees(self.robot_angle):.1f}°",
                throttle_duration_sec=0.5
            )

            self.update_display()

        except Exception as e:
            self.get_logger().error(f"Error en IMU callback: {str(e)}")

    def update_display(self):
        """Renderiza el mapa y el robot"""
        self.screen.fill((0, 0, 0))

        # Dibujar mapa
        for y in range(0, self.height, self.cell_size):
            for x in range(0, self.width, self.cell_size):
                if np.any(self.terrain_map[y, x] != 0):
                    pygame.draw.rect(
                        self.screen,
                        self.terrain_map[y, x],
                        (x, y, self.cell_size, self.cell_size)
                    )

        # Dibujar robot (triángulo orientado según yaw)
        points = [
            (self.robot_pos[0] + 15 * math.cos(self.robot_angle),
             self.robot_pos[1] - 15 * math.sin(self.robot_angle)),
            (self.robot_pos[0] + 8 * math.cos(self.robot_angle + 2.5),
             self.robot_pos[1] - 8 * math.sin(self.robot_angle + 2.5)),
            (self.robot_pos[0] + 8 * math.cos(self.robot_angle - 2.5),
             self.robot_pos[1] - 8 * math.sin(self.robot_angle - 2.5))
        ]
        pygame.draw.polygon(self.screen, (255, 255, 255), points)

        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = IMUMapper()

    try:
        clock = pygame.time.Clock()
        running = True
        while running:
            rclpy.spin_once(node, timeout_sec=0.01)
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    running = False
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()