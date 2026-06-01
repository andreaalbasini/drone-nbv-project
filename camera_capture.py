#!/usr/bin/env python3
import os
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3  
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ==========================================
# CONFIGURAZIONE PARAMETRI
# ==========================================
CHESSBOARD_SIZE = (7, 5)  
SAVE_PATH = os.path.expanduser('~/ros2_ws_AA/calibration_data/')
CAPTURE_INTERVAL = 2.0    
# ==========================================

class SiyiAutoCaptureNode(Node):
    def __init__(self):
        super().__init__('siyi_auto_capture_node')
        self.bridge = CvBridge()
        
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
            self.get_logger().info(f"Creata cartella in: {SAVE_PATH}")

        # Configurazione Gimbal a 0
        self.gimbal_pub = self.create_publisher(Vector3, '/siyi_a8/gimbal_control', 10)
        self.gimbal_timer = self.create_timer(1.0, self.lock_gimbal_at_zero)

        # Configurazione del profilo QoS compatibile (Best Effort + Sensor Data)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Accetta anche flussi video veloci/IP
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Sottoscrizione al flusso video (Verifica che '/camera/image_raw' sia identico a 'ros2 topic list')
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw',  
            self.image_callback, 
            qos_profile
        )
        
        self.current_frame = None
        self.img_count = 0
        self.first_frame_received = False
        self.last_capture_time = time.time()
        
        self.get_logger().info("NODO AVVIATO. In attesa del primo frame video...")

    def lock_gimbal_at_zero(self):
        msg = Vector3()
        msg.x = 0.0  
        msg.y = 0.0  
        msg.z = 0.0  
        self.gimbal_pub.publish(msg)

    def image_callback(self, msg):
        # Log di debug per capire se i dati stanno arrivando
        if not self.first_frame_received:
            self.get_logger().info("--> [OK] Primo frame ricevuto! Apertura finestra video...")
            self.first_frame_received = True

        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            display_frame = self.current_frame.copy()
            
            gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
            
            current_time = time.time()
            
            if ret:
                cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners, ret)
                time_elapsed = current_time - self.last_capture_time
                time_remaining = max(0.0, CAPTURE_INTERVAL - time_elapsed)
                
                if time_remaining == 0:
                    self.save_image()
                    self.last_capture_time = current_time
                    cv2.putText(display_frame, "FOTO SALVATA!", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, f"Pronto tra: {time_remaining:.1f}s", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
            else:
                cv2.putText(display_frame, "Cerca scacchiera...", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(display_frame, f"Totale foto: {self.img_count}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow("SIYI A8 - Capture Mode", display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Errore conversione: {str(e)}")

    def save_image(self):
        self.img_count += 1
        filename = os.path.join(SAVE_PATH, f"calib_{self.img_count:02d}.jpg")
        cv2.imwrite(filename, self.current_frame)
        self.get_logger().info(f"[AUTO] Foto {self.img_count} salvata!")

def main(args=None):
    rclpy.init(args=args)
    node = SiyiAutoCaptureNode()
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
