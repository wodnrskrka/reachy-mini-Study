import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

# ===== 사용자 설정 =====
MODEL_PATH = "/home/naseungwon/reachy_mini/yolo_detect/best.pt"
EMOTION_LABELS = ['anger', 'fear', 'happy', 'neutral', 'sad'] 
CONFIDENCE_THRESHOLD = 0.4

class EmotionDetector(Node):
    def __init__(self):
        super().__init__('emotion_detector_node')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/emotion_detector/image', 10)
        self.label_pub = self.create_publisher(String, '/detected_emotion', 10)
        self.get_logger().info("✅ 감정 감지 노드가 시작되었습니다.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=frame, imgsz=640, conf=CONFIDENCE_THRESHOLD, verbose=False)

        result_frame = frame.copy()
        detected_emotions = []

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            label = EMOTION_LABELS[cls_id]
            detected_emotions.append(label)

            cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(result_frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if detected_emotions:
            msg = String()
            msg.data = ', '.join(detected_emotions)
            self.label_pub.publish(msg)

        # 이미지 결과 publish
        out_msg = self.bridge.cv2_to_imgmsg(result_frame, encoding='bgr8')
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmotionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
