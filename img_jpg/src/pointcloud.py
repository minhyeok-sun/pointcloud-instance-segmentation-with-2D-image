#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
import time


#p_mat = np.array([[0,0,1],[-0.001481341,0,0.8890002],[0,-0.001481341,0.5741718]]) #for zed camera
p_mat = np.array([[0,0,1],[-0.00160981,0,0.5177980],[0,-0.00160981,0.390396]]) #for intel realsense camera

class seg_3d():
    def __init__(self,name):
        self.name = name #name of node and npy file
        self.pub = rospy.Publisher(name, PointCloud, queue_size = 5)
        self.seg_data = np.load('/home/sun/catkin_ws/src/img_jpg/data/'+ self.name + '.npy') #seg result npy
        self.custom_point = PointCloud() # data to send
        self.custom_point.header.frame_id = "camera_link"
        self.channel = ChannelFloat32()

    def projection(self,downscale): 
        count = 0
        for j in range(self.seg_data.shape[0]/downscale - 1):
            data = Point32()

            # (X,Y,Z) = inverse_projection * inverse_rotation * (imgx * depth, imgy * depth, depth)
            convert_data = np.dot(p_mat,self.seg_data[downscale*j]) 

            data.x = convert_data[0][0]/1000.0 # mm to meter
            data.y = convert_data[1][0]/1000.0
            data.z = convert_data[2][0]/1000.0
            self.custom_point.points.append(data)
            count += 1
        print('Number of pointcloud of "'+ self.name +'" is ' + str(count))

    def publish(self):
        self.pub.publish(self.custom_point)


if __name__ == '__main__':
    rospy.init_node('seg_3d')
    rate = rospy.Rate(5) #frequency

    a = seg_3d('matrix_23_0')
    a.projection(10)
    b = seg_3d('matrix_23_1')
    b.projection(10)
    c = seg_3d('matrix_23_2')
    c.projection(10)

    while not rospy.is_shutdown():
        a.publish()
        b.publish()
        c.publish()
        rate.sleep()

    

