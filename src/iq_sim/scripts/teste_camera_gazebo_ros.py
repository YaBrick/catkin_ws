#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


    
class WebcamProcessor:
    def __init__(self):
    
        rospy.init_node('webcam_processor', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber('/webcam', Image, self.callback)
        rospy.loginfo("WebcamProcessor node started, waiting for images...")
        rospy.spin()

    def callback(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            print("Nao foi possivel obter frame da camera")
            rospy.logerr(f"CvBridge Error: {e}")
            return
            
            cv2.imwrite("imagem_capturada.jpg", frame)

    

if __name__ == '__main__':
    try:
        WebcamProcessor()
    except rospy.ROSInterruptException:
        pass

