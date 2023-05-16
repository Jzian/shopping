 #!/usr/bin/env python
import sys
import rospy
import numpy as np
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
import time

def marker_det_client(id,size):   
    rospy.wait_for_service('Marker_det')
    try:
        client = rospy.ServiceProxy('Marker_det', MarkerDetResult)
        resp = client(id, size)
        return resp
    except rospy.ServiceException as e:
         print("Service call failed: %s"%e)
if __name__ == "__main__":
    i=0
    rospy.init_node('Marker_client_node')
    while(i<4):
        resp=marker_det_client(i,0.03)
        print(resp)
        i=i+1
        time.sleep(1)

