#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include<PID_v1.h>

ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped P;
//ros::Publisher tester("testing_topic", &P);

const double wheel_radius = 0.065;//in m
const double wheel_gap=0.395;
int ticks = 2100;

//Controller params
float demandlinear=0;
float demandangular=0;
double demand_speed_left=0;
double demand_speed_right=0;
double current_speed_left=0;
double current_speed_right=0;

double left_kp = 3000 , left_ki = 0 , left_kd =0;             // modify for optimal performance
double right_kp = 3000 , right_ki = 0 , right_kd = 0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandlinear = twist.linear.x;
  demandangular = twist.angular.z;
  //Serial.println(demandlinear,4);
}


void current_vel_cb( const geometry_msgs::Twist& twist){
  current_speed_left = twist.linear.x;
  current_speed_right = twist.linear.y;
  //Serial.println("callback");
  //Serial.println(current_speed_left,4);
}

ros::Subscriber<geometry_msgs::Twist> sub_demand("/cmd_vel", &cmd_vel_cb );
ros::Subscriber<geometry_msgs::Twist> sub_currentvel("/agv_currentvel", &current_vel_cb);

//Bluetooth Code Begins
char t;

int ENB = 5;
int IND = 9;
int Q = (160);
int INC = 6;

int ENA = 7;
int INA = 8;
int INB = 13;


void setup() {
  nh.initNode();
  nh.subscribe(sub_demand);
  nh.subscribe(sub_currentvel);
  //nh.advertise(tester); 

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(0, 160);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(0, 160);
    
  pinMode(ENB,OUTPUT);   
  pinMode(IND,OUTPUT);   
  pinMode(INC,OUTPUT);   
  pinMode(ENA,OUTPUT);   
  pinMode(INA,OUTPUT);  
  pinMode(INB,OUTPUT);
//  
  
  //Bluetooth Code Ends
  Serial.begin(115200);

 
}
 
void loop() {

    nh.spinOnce();
    //delay(1);

    demand_speed_left = demandlinear - (demandangular*wheel_gap/2);
    demand_speed_right = demandlinear + (demandangular*wheel_gap/2);

    left_setpoint = demand_speed_left;  
    right_setpoint = demand_speed_right;

    left_input = current_speed_left;  
    right_input = current_speed_right;

    leftPID.Compute();
    rightPID.Compute();

    if (left_setpoint < 0){
      digitalWrite(INC,HIGH);
      digitalWrite(IND,LOW);}
    else{
      digitalWrite(INC,LOW);
      digitalWrite(IND,HIGH);
    }
  
    if (right_setpoint < 0){
      digitalWrite(INA,LOW);
      digitalWrite(INB,HIGH);}
    else{
      digitalWrite(INA,HIGH);
      digitalWrite(INB,LOW);
    }

    if (abs(left_setpoint-left_input)> 0.05){
      analogWrite(ENA,left_output);}
    else{
      analogWrite(ENA,0);}

     if (abs(left_setpoint-left_input)> 0.05){
      analogWrite(ENB,left_output);}
    else{
      analogWrite(ENB,0);}

   

    
//    P.header.stamp = nh.now();
//    P.header.frame_id = "/test";
//  
//    P.vector.x=demand_speed_left;
//    P.vector.y=demand_speed_right;
//    tester.publish(&P);

}
