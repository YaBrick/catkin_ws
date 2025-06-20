#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WebcamProcessor:
    def __init__(self):
        rospy.init_node('webcam_processor', anonymous=True)
        self.bridge = CvBridge()
        # Подписываемся на топик
        rospy.Subscriber('/webcam/image_raw', Image, self.callback)
        rospy.loginfo("WebcamProcessor node started, waiting for images...")
        rospy.spin()

    def callback(self, msg):
        try:
            # Конвертация ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Пример обработки — детектирование границ
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        print("Captured")
        cv2.imwrite("imagem_capturada.jpg", cv_image)

if __name__ == '__main__':
    try:
        WebcamProcessor()
    except rospy.ROSInterruptException:
        pass
