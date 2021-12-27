#!/usr/bin/env python

import roslib; roslib.load_manifest('dmie_agv_odom')
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
#from scipy.interpolate import interp1d
#from scipy.interpolate import InterpolatedUnivariateSpline

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
import tf as tf


#from agv_odom import henc

rospy.init_node('dmie_agv_odom', anonymous=True)
current_encoder_left=0
current_encoder_right=0
previous_encoder_left=0
previous_encoder_right=0
encoder_wraps_x=0
encoder_wraps_y=0
previous_tmpx=0
previous_tmpy=0

current_time=0
previous_time=0

positionx=0
positiony=0
orientz=0
increasex=0
increasey=0

odom_message=Odometry()
vel_message=Twist()

odom_cnt=0

wheel_radius = 0.048; #in m
wheel_gap=0.37;
ticks_per_rev = 2100;


def wrapTo2Pi(angles):

    tmp = np.arctan2(np.sin(angles), np.cos(angles))
    return float(np.where(tmp<0 , 2*np.pi+tmp, tmp))
  
  #wrapped=(2*np.pi + angles) * (angles < 0) + angles*(angles >= 0)
  
  #return wrapped

def handle_dat_encoder(encoder_x,encoder_x_prev, encoder_y, encoder_y_prev):
    global encoder_wraps_x, encoder_wraps_y, increasex, increasey

    tmpx=encoder_x
    tmpy=encoder_y

    # process left
    
    # calculate diff
    diff = encoder_x - encoder_x_prev

    #print("current x : ", encoder_x, "previous x :", encoder_x_prev, "diff : ", diff)


    # check wrap around
    if abs(diff) > (65000):
        # if positive wrap around, increase
        if encoder_x < encoder_x_prev:
            print("Positive LEFT")
            encoder_wraps_x = encoder_wraps_x + 1
            increasex=1
        # else decrease
        else:
            print("Negative LEFT")
            encoder_wraps_x = encoder_wraps_x - 1
            increasex=0
                 

    if abs(encoder_wraps_x) > 0:

        if increasex==1:
            tmpx = 32767*(2*encoder_wraps_x-1) + abs(-32768-encoder_x)
        else:
            tmpx = -32768*(2*encoder_wraps_x-1) - (32767-encoder_x) 

    else:  
        tmpx=encoder_x


    #print("tmpX : ", tmpx)

    # process y

    #calculate diff
    diff = encoder_y - encoder_y_prev
    #print("current y : ", encoder_y, "previous y :", encoder_y_prev, "diff : ", diff)

    # check wrap around
    if abs(diff) > (65000):
        #if positive wrap around, increase
        if encoder_y < encoder_y_prev:
            print("Positive RIGHT")
            encoder_wraps_y = encoder_wraps_y + 1
            increasey=1

        # else decrease
        else:
            print("Negative RIGHT")
            encoder_wraps_y = encoder_wraps_y - 1
            increasey=0
            
    if abs(encoder_wraps_y) > 0:

        if increasey==1:
            tmpy = 32767*(2*encoder_wraps_y-1) + abs(-32768-encoder_y)
        else:
            tmpy = -32768*(2*encoder_wraps_y-1) - (32767-encoder_y) 

    else:  
        tmpy=encoder_y
    #print("tmpY : ", tmpy)

    #print("encoder mean : ", encoder_mean)

    return (tmpx, tmpy)




def callback(data):
    global current_encoder_left, current_encoder_right, previous_encoder_left, previous_encoder_right, odom_cnt, odom_message, current_time, previous_time
    global wheel_radius, wheel_gap,ticks_per_rev, positionx, positiony, orientz,previous_tmpx,previous_tmpy
    #rospy.loginfo("Recieved %f, %f ", data.vector.x,data.vector.y)
    odom_broadcaster = tf.TransformBroadcaster()

    previous_time=current_time
    current_time=data.header.stamp
    
    time_gap=current_time.to_sec()-previous_time.to_sec()

    previous_encoder_left=current_encoder_left
    previous_encoder_right=current_encoder_right


    current_encoder_left=data.vector.x
    current_encoder_right=data.vector.y

    (tmpx,tmpy)=handle_dat_encoder(current_encoder_left,previous_encoder_left,current_encoder_right,previous_encoder_right)

    odom_pub = rospy.Publisher('agv_odometry', Odometry, queue_size=10)
    currentvel_pub = rospy.Publisher('agv_currentvel', Twist, queue_size=10)

    left_dis_gap = np.float64((tmpx-previous_tmpx)/ticks_per_rev)*2*np.pi*wheel_radius;
    right_dis_gap = np.float64((tmpy-previous_tmpy)/ticks_per_rev)*2*np.pi*wheel_radius;

    previous_tmpx=tmpx
    previous_tmpy=tmpy

    velocity_left=left_dis_gap/time_gap
    velocity_right=right_dis_gap/time_gap

    deltarotz=((left_dis_gap-right_dis_gap)/wheel_gap)
    trans=(left_dis_gap+right_dis_gap)/2


    
    orientz=wrapTo2Pi(orientz+deltarotz)

    if orientz < 0.00001:

        orientz = 0



    #print("orientz : ", orientz)

    deltax=trans*np.cos(orientz)
    deltay=trans*np.sin(orientz)

    positionx=positionx+deltax
    positiony=positiony+deltay

    odom_cnt = odom_cnt + 1

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, orientz)

    print("orientz : ", odom_quat)

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

    odom_message.pose.pose.orientation.x=odom_quat[0]
    odom_message.pose.pose.orientation.y=odom_quat[1]
    odom_message.pose.pose.orientation.z=odom_quat[2]
    odom_message.pose.pose.orientation.w=odom_quat[3]

    odom_pub.publish(odom_message)
    
    #vel_message.header.stamp = current_time
    #vel_message.header.seq = odom_cnt

    #vel_message.header.frame_id = 'odom'
   
    vel_message.linear.x=velocity_left
    vel_message.linear.y=velocity_right
    
    currentvel_pub.publish(vel_message)


    
def encoder_listen():
   rospy.loginfo("STARTED!!")


   rospy.Subscriber("encoder_ticks", Vector3Stamped, callback)
   
   #encoder_pub()

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':

    encoder_listen()
    
    
