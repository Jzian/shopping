#!/usr/bin/env python

import os
import sys
root_path = os.path.dirname(os.path.abspath(__file__))
print(root_path)
root_path_1 = '/'.join(root_path.split('/')[:-1])
root_path_2 = '/'.join(root_path.split('/')[:-2])
root_path_3 = '/'.join(root_path.split('/')[:-3])
sys.path.append(root_path_1)
sys.path.append(root_path_2)
sys.path.append(root_path_3)

import numpy as np
import rospy
from sensor_msgs.msg import Image,CameraInfo
import message_filters
from geometry_msgs.msg import Transform
from cv_bridge import CvBridge
import cv2
from IPython import embed
from geometry_msgs.msg import TransformStamped
import tf_conversions
import argparse

from scipy.spatial.transform import Rotation
from std_srvs.srv import *
from shopping.srv import MarkerDetResult,MarkerDetResultRequest,MarkerDetResultResponse

import threading


from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R


image_path = '../src/images/'
depth_path ='../src/depths/'
image_topic_name = '/camera/color/image_raw'
# depth_topic_name = '/camera/aligned_depth_to_color/image_raw'
camera_info = '/camera/color/camera_info'


image_index = 0


class ShoppingService:
    def __init__(self, req):
        self.id = req.id
        self.size = req.size
        self.flag = True
        self.matrix_coefficients=np.array([[320.1253662109375,0,318.70709228515625],[0,319.3437194824219,183.09527587890625],[0,0,1.0]])
        self.distortion_coefficients=np.array([-0.05622655153274536, 0.06800640374422073, 0.0006116469157859683, 0.0007112481398507953, -0.021988557651638985])
        self.process_done = False
        self.bridge = CvBridge()
        add_thread = threading.Thread(target=self.thread_job)
        add_thread.start()
        while(True):
            if self.process_done == True:
                break

    def thread_job(self):
        self.color = message_filters.Subscriber(image_topic_name, Image)
        self.camerainfo = message_filters.Subscriber(camera_info, CameraInfo)
        self.color_caminfo = message_filters.TimeSynchronizer([self.color, self.camerainfo], 1)
        self.color_caminfo.registerCallback(self.callback)
        rospy.spin()

    def callback(self, image, camera_info):
        if self.flag:
            self.cv_color = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            self.matrix_coefficients = np.array(camera_info.K).reshape([3, 3])
            print(self.matrix_coefficients)
            self.distortion_coefficients = np.array(camera_info.D)
            print(self.distortion_coefficients )
            # rgb_undist = cv2.undistort(self.cv_color, self.matrix_coefficients,self.distortion_coefficients)
            # self.cv_depth = self.bridge.imgmsg_to_cv2(depth, '16UC1')

            global image_index
            # rospy.loginfo('Receive and save images ')
            # image_name = image_path + 'image_' + str(image_index)+ ".png"
            # depth_name = depth_path + 'depth_' + str(image_index)+ ".png"
            # print('image: ', image_name)
            # print('depth: ', depth_name)
            image_index = image_index + 1
            # cv2.imwrite(image_name, self.cv_color)
            # cv2.imwrite(depth_name, self.cv_depth)
            self.flag = False
            rot_vec, trans, idcount, idexist = self.detect_aruco(self.cv_color)
            self.output = MarkerDetResultResponse()
            if idexist:                
                trans=np.ravel(trans)
                rot_mat,_=cv2.Rodrigues(rot_vec)
                rotat = R.from_matrix(rot_mat).as_matrix()
                print(rotat,trans)                                
                self.output.rotat_0 = rotat[0][0]
                self.output.rotat_1 = rotat[0][1]
                self.output.rotat_2 = rotat[0][2]
                self.output.rotat_3 = rotat[1][0]
                self.output.rotat_4 = rotat[1][1]
                self.output.rotat_5 = rotat[1][2]
                self.output.rotat_6 = rotat[2][0]
                self.output.rotat_7 = rotat[2][1]
                self.output.rotat_8 = rotat[2][2]
                self.output.trans_x = trans[0]
                self.output.trans_y = trans[1]
                self.output.trans_z = trans[2]
                self.output.idcount = idcount
                self.output.idexist = 1
            elif not idcount:
                self.output.rotat_0 = 0
                self.output.rotat_1 = 0
                self.output.rotat_2 = 0
                self.output.rotat_3 = 0
                self.output.rotat_4 = 0
                self.output.rotat_5 = 0
                self.output.rotat_6 = 0
                self.output.rotat_7 = 0
                self.output.rotat_8 = 0
                self.output.trans_x = 0
                self.output.trans_y = 0
                self.output.trans_z = 0
                self.output.idcount = idcount
                self.output.idexist = 0
            else:
                self.output.rotat_0 = 0
                self.output.rotat_1 = 0
                self.output.rotat_2 = 0
                self.output.rotat_3 = 0
                self.output.rotat_4 = 0
                self.output.rotat_5 = 0
                self.output.rotat_6 = 0
                self.output.rotat_7 = 0
                self.output.rotat_8 = 0
                self.output.trans_x = 0
                self.output.trans_y = 0
                self.output.trans_z = 0
                self.output.idcount = idcount
                self.output.idexist = 0
        self.process_done = True

    def detect_aruco(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        # parameters = aruco.DetectorParameters()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        
        print(ids)

        if  ids is None:
            return None,None,0,0
        else:
            if self.id in ids:
                for i in range(len(ids)):
                    if (ids[i]==self.id):
                        rvec,tvec,_=cv2.aruco.estimatePoseSingleMarkers(corners[i], self.size, self.matrix_coefficients,self.distortion_coefficients)
                return rvec,tvec,len(ids),1
            else: 
                return None,None,len(ids),0
            



    
def markerdet(req):
    serviceNode = ShoppingService(req)
    return serviceNode.output

if __name__ == '__main__':
     
    try:
        rospy.init_node('Marker_service_node')
        rospy.Service('Marker_det', MarkerDetResult, markerdet)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    