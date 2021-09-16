#!/usr/bin/env python

import roslib; roslib.load_manifest('agv_odom')
import rospy
import copy
import time
import os, sys
import subprocess 
import signal
import re
from array import *
import numpy as np
from numpy.linalg import multi_dot
from numpy.linalg import inv
from numpy.linalg import cond
import scipy.io #For matlab
from decimal import Decimal
from scipy.interpolate import interp1d
from scipy.interpolate import InterpolatedUnivariateSpline

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped 
import tf as tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray 

from agv_odom import henc

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       pub.publish(hello_str)
       rate.sleep()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('listener', anonymous=True)

   rospy.Subscriber("chatter", String, callback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
  listener()