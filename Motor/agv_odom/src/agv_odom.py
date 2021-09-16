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
#import scipy.io #For matlab
from decimal import Decimal
from scipy.interpolate import interp1d
from scipy.interpolate import InterpolatedUnivariateSpline

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import tf as tf


#from agv_odom import henc

rospy.init_node('agv_odom', anonymous=True)
current_encoder_left=0
current_encoder_right=0
previous_encoder_left=0
previous_encoder_right=0

current_time=0
previous_time=0

positionx=0
positiony=0
orientz=0

odom_message=Odometry()
odom_cnt=0

wheel_radius = 0.048; #in m
wheel_gap=0.37;
ticks_per_rev = 2100;




def callback(data):
    global current_encoder_left, current_encoder_right, previous_encoder_left, previous_encoder_right, odom_cnt, odom_message, current_time, previous_time
    global wheel_radius, wheel_gap,ticks_per_rev, positionx, positiony, orientz
    #rospy.loginfo("Recieved %f", data.vector.x)
    odom_broadcaster = tf.TransformBroadcaster()

    previous_time=current_time
    current_time=data.header.stamp

    previous_encoder_left=current_encoder_left
    previous_encoder_right=current_encoder_right


    current_encoder_left=data.vector.x
    current_encoder_right=data.vector.y

    odom_pub = rospy.Publisher('agv_odometry', Odometry, queue_size=10)

    left_dis_gap = np.float64((current_encoder_left-previous_encoder_left)/ticks_per_rev)*2*np.pi*wheel_radius;
    right_dis_gap = np.float64((current_encoder_right-previous_encoder_right)/ticks_per_rev)*2*np.pi*wheel_radius;

    deltarotz=(left_dis_gap-right_dis_gap)/wheel_gap
    trans=(left_dis_gap+right_dis_gap)/2

    deltax=trans*np.cos(deltarotz)
    deltay=trans*np.sin(deltarotz)

    positionx=positionx+deltax
    positiony=positiony+deltay
    orientz=orientz+deltarotz

    odom_cnt = odom_cnt + 1

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, orientz)

    odom_broadcaster.sendTransform(
        (positionx, positiony, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )


    odom_message.header.stamp = current_time
    odom_message.header.seq = odom_cnt

    odom_message.header.frame_id = 'odom'
    odom_message.pose.pose.position.x=positionx
    odom_message.pose.pose.position.y=positiony

    odom_pub.publish(odom_message)


    
def encoder_listen():


   rospy.Subscriber("encoder_ticks", Vector3Stamped, callback)
   #encoder_pub()

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':

    encoder_listen()
    
    
