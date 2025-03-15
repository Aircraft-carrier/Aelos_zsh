#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        # 初始化节点
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/processed_image', Image, queue_size=10)
        
        # 订阅图像话题
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        
        # 定义颜色范围（HSV）
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {str(e)}")
            return
        
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 创建掩膜
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 查找轮廓
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_contours = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            
            # 圆形度检测
            (x,y), radius = cv2.minEnclosingCircle(cnt)
            circle_area = np.pi * (radius**2)
            circularity = area / circle_area
            if circularity < 0.4:
                continue
            
            # 获取边界框
            x,y,w,h = cv2.boundingRect(cnt)
            
            # 绘制结果
            cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 2)
            center = (x + w//2, y + h//2)
            cv2.putText(cv_image, f"({center[0]}, {center[1]})", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            rospy.loginfo(f"发现红色物体: 中心坐标 ({center[0]}, {center[1]})")
            valid_contours += 1
        
        # 发布处理后的图像
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"处理后图像转换失败: {str(e)}")

if __name__ == '__main__':
    try:
        subscriber = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass