#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import sensor_msgs.point_cloud2 as pc2

 
class img2jpg():
    def __init__(self,topic,name,color):
        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
        self.bridge = CvBridge()
        self._sub = rospy.Subscriber(topic, Image, self.callback, queue_size=1)
        self.color = color #0:bgr8, 1:8uc1
        self.count = 0
        self.path = "/home/sun/catkin_ws/src/img_jpg/data/"+ name + "/"
        os.mkdir(self.path)

    def callback(self, image_msg):
        if self.color == 0: # color(normal)
            if self.selecting_sub_image == "compressed":
                np_arr = np.fromstring(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif self.selecting_sub_image == "raw":
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        else: #gray(depth)
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "16UC1") 
            np.save(self.path + str(self.count), cv_image)
            print(cv_image/float(1000.000))


        cv2.imwrite(self.path + str(self.count) +".jpg", cv_image)
        print('success', image_msg.header.seq)
        self.count += 1
        

    def main(self):
        rospy.spin()
 
if __name__ == '__main__':
    rospy.init_node('image_jpg_node')
    color = img2jpg('/camera/color/image_raw',"color",0)
    depth = img2jpg('/camera/aligned_depth_to_color/image_raw',"depth",1) 
    depth.main()
    depth.main()






