#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& toggle_msg){
 Serial.println("callback");
}

ros::Subscriber<geometry_msgs::Twist> sub("agv_currentvel", &messageCb );

void setup()
{

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
