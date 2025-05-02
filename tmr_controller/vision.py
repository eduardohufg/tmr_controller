import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int16
import cv2
from cv_bridge import CvBridge

class ColorObjectTracker(Node):
    def __init__(self):
        super().__init__('color_object_tracker')

        # Publishers
        self.offset_pub   = self.create_publisher(Point,  '/object_offset',   10)
        self.detected_pub = self.create_publisher(Bool,   '/object_detected', 10)
        self.centered_pub = self.create_publisher(Bool,   '/object_centered', 10)
        self.color_pub    = self.create_publisher(Int16,  '/object_color',    10)

        # Timer (≈100 Hz)
        self.create_timer(0.01, self.process_frame)

        # Params
        self.threshold_centering = 20   # píxeles
        self.min_area            = 10000  # píxeles²

        # Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara 0')
            raise RuntimeError('Camera open failed')

        self.bridge = CvBridge()
        self.get_logger().info('Color Object Tracker Node Started')

    # ---------- Main loop ----------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame no disponible')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rangos HSV
        ranges = {
            1: [(0, 100, 100),  (10, 255, 255),  (160, 100, 100), (179, 255, 255)],  # rojo
            2: [(40, 70, 70),   (80, 255, 255)],                                     # verde
            3: [(100, 150, 0),  (140, 255, 255)]                                     # azul
        }

        detected, centered = False, False
        offset_x, offset_y = 0, 0
        detected_color     = 0     # 0 = ninguno

        # Centro del frame
        fcy, fcx = frame.shape[0] // 2, frame.shape[1] // 2

        for color_id, bounds in ranges.items():
            # Crear máscara (rojo tiene dos rangos)
            if color_id == 1:
                mask = (cv2.inRange(hsv, bounds[0], bounds[1]) |
                        cv2.inRange(hsv, bounds[2], bounds[3]))
            else:
                mask = cv2.inRange(hsv, bounds[0], bounds[1])

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self.min_area:
                continue

            # --- Objeto válido ---
            detected = True
            M = cv2.moments(largest)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            offset_x, offset_y = cx - fcx, cy - fcy

            # Dibujar contorno y centro
            cv2.drawContours(frame, [largest], -1, (255, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

            if abs(offset_x) < self.threshold_centering and abs(offset_y) < self.threshold_centering:
                centered       = True
                detected_color = color_id
            else:
                detected_color = 0
            break                      # ya encontramos un objeto válido

        # Dibujar centro del frame
        cv2.drawMarker(frame, (fcx, fcy), (0, 0, 0), cv2.MARKER_CROSS, 15, 2)

        # Mostrar
        cv2.imshow('Color detector', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

        # -------- Publicaciones --------
        self.offset_pub.publish(Point(x=float(offset_x), y=float(offset_y), z=0.0))
        self.detected_pub.publish(Bool(data=detected))
        self.centered_pub.publish(Bool(data=centered))
        self.color_pub.publish(Int16(data=detected_color))

# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
