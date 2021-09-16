import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from arf_msgs.msg import encoder
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import TransformStamped 
import tf as tf
import tf2_ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



def publishpose(pose_list,GTmap):
      
  #global pose_list, GTmap
  
  final_pose=pose_list[-1]
  pose_msg=PoseWithCovarianceStamped()

  pose_msg.header.stamp = rospy.Time.now()
  pose_msg.header.seq = pose_list.size
  pose_msg.header.frame_id = 'EKF'
  pose_msg.pose.pose.position.x=final_pose.X
  pose_msg.pose.pose.position.y=final_pose.Y
  quaternion = tf.transformations.quaternion_from_euler(0, 0, final_pose.Phi)
  pose_msg.pose.pose.orientation.x = quaternion[0]
  pose_msg.pose.pose.orientation.y = quaternion[1]
  pose_msg.pose.pose.orientation.z = quaternion[2]
  pose_msg.pose.pose.orientation.w = quaternion[3]
  pose_msg.pose.covariance[0]=final_pose.P[0,0]
  pose_msg.pose.covariance[7]=final_pose.P[1,1]
  pose_msg.pose.covariance[14]=0
  pose_msg.pose.covariance[21]=0
  pose_msg.pose.covariance[28]=0
  pose_msg.pose.covariance[35]=final_pose.P[2,2]
  
  EKF_pub=rospy.Publisher('/EKF_pose', PoseWithCovarianceStamped, queue_size=10) 
  EKF_pub.publish(pose_msg)
  #print(final_pose.X,final_pose.Y,final_pose.Phi)
  
  markerarray=MarkerArray()
  marker=Marker()
  
  for m in range(0,GTmap.shape[1]):
    marker.header.frame_id = 'EKF'
    marker.header.stamp=rospy.Time.now()
    marker.pose.position.x=GTmap[0,m]
    marker.pose.position.y=GTmap[1,m]
    marker.pose.position.z=0
    marker.type = marker.SPHERE
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0.0
    marker.lifetime=rospy.Duration(0)
    marker.id=m
    markerarray.markers.append(marker)
    marker_pub=rospy.Publisher('/map_marks', MarkerArray, queue_size=10) 
    marker_pub.publish(markerarray)
    
  odom_message=Odometry()
  
  odom_message.header.stamp = rospy.Time.now()
  odom_message.header.seq = pose_list.size
  odom_message.header.frame_id = 'EKF'
  odom_message.pose=pose_msg.pose
  odom_pub=rospy.Publisher('/EKF_odom', Odometry, queue_size=10) 
  odom_pub.publish(odom_message)
  
  br = tf2_ros.TransformBroadcaster()
  t =TransformStamped()
  
  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "EKF"
  t.child_frame_id = "base_link"
  t.transform.translation.x = final_pose.X#-0.2*np.cos(final_pose.Phi)
  t.transform.translation.y = final_pose.Y#-0.2*np.cos(final_pose.Phi)
  t.transform.translation.z = 0.0
  t.transform.rotation.x = quaternion[0]
  t.transform.rotation.y = quaternion[1]
  t.transform.rotation.z = quaternion[2]
  t.transform.rotation.w = quaternion[3]
  
  br.sendTransform(t)
  