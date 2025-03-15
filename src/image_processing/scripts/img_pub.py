#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.cap = cv2.VideoCapture(0)  # 使用默认摄像头
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头!")
            exit(1)
        
        self.rate = rospy.Rate(30)  # 30Hz发布频率
        rospy.loginfo("摄像头发布节点已启动")

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.pub.publish(ros_image)
                except Exception as e:
                    rospy.logerr(f"图像转换失败: {str(e)}")
                self.rate.sleep()
            else:
                rospy.logwarn("未接收到摄像头帧")

if __name__ == '__main__':
    try:
        publisher = ImagePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass