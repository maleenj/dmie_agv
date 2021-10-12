import rospy
import numpy as np
import scipy.io #For matlab
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

rospy.init_node('ams_ekf_v1', anonymous=True,log_level=rospy.DEBUG)
mat= scipy.io.loadmat('/home/maleen/git/ams_scooter/ams_ekf/src/demomaploop12.mat')

GTmap=mat['finmap']

def publishmap(GTmap):

 
      
  markerarray=MarkerArray()
  marker=Marker()
  
  for m in range(0,GTmap.shape[1]):
    marker.header.frame_id = 'mapvizz'
    marker.header.stamp=rospy.Time.now()
    marker.pose.position.x=GTmap[0,m]
    marker.pose.position.y=GTmap[1,m]
    marker.pose.position.z=0
    marker.type = marker.SPHERE
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.color.a = 1
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0.0
    marker.lifetime=rospy.Duration(0)
    marker.id=m
    markerarray.markers.append(marker)
    marker_pub=rospy.Publisher('/map_marks_viz', MarkerArray, queue_size=10) 
    marker_pub.publish(markerarray)

def main(args):
   global GTmap
   publishmap(GTmap)
   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
      
 
if __name__ == '__main__':
    main(sys.argv)