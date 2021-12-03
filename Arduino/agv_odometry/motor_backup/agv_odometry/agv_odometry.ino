#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include<PID_v1.h>

ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks ther direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 10

//
const double wheel_radius = 0.048;//in m
const double wheel_gap=0.37;
int ticks = 2100;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;
volatile int prev_left_wheel_tick_count = 0;
volatile int prev_right_wheel_tick_count = 0;

//speeds 
double v_l = 0;
double v_r = 0;
float left_dis_gap = 0;
float right_dis_gap = 0;
float total_left_distance = 0;
float total_right_distance = 0;

double positionX =0;
double positionY=0;
double yaw=0;


//interval for measurements
int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
long timegap = 0;

//Controller params
float demandx=0;
float demandz=0;
double demand_speed_left;
double demand_speed_right;
int ENB = 5;
int IND = 9;
int INC = 6;

int ENA = 7;
int INA = 8;
int INB = 13;

double left_kp = 3.8 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 4 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
  //Serial.println(demandx,4);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_vel_cb );
 
void setup() {

  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
  // Open the serial port at 9600 bps
  Serial.begin(115200); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  pinMode(ENB,OUTPUT);   //left motors forward
  pinMode(IND,OUTPUT);   //left motors reverse
  pinMode(INC,OUTPUT);   //right motors forward
  pinMode(ENA,OUTPUT);   //right motors reverse
  pinMode(INA,OUTPUT);   //Led
  pinMode(INB,OUTPUT);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-140, 140);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-140, 140);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
}
 
void loop() {
 
  // Record the time
  currentMillis = millis();
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
    timegap= currentMillis -previousMillis;
    previousMillis = currentMillis;
    left_dis_gap = ((left_wheel_tick_count - prev_left_wheel_tick_count)/ticks)*2*PI*wheel_radius;
    right_dis_gap = ((right_wheel_tick_count - prev_right_wheel_tick_count)/ticks)*2*PI*wheel_radius;
    v_l = 1000*(left_dis_gap/timegap);// (m/s)
    v_r = 1000*(right_dis_gap/timegap);

    demand_speed_left = demandx - (demandz*wheel_gap/2);
    demand_speed_right = demandx + (demandz*wheel_gap/2);

    //run_PID(v_l,v_r,timegap,demand_speed_left,demand_speed_right)

    left_setpoint = demand_speed_left;  
    right_setpoint = demand_speed_right;

    left_input = v_l;  
    right_input = v_r;

    leftPID.Compute();
    rightPID.Compute();

//    analogWrite(ENA,left_output*100);
//    analogWrite(ENB,right_output*100);
//    digitalWrite(IND,HIGH);
//    digitalWrite(INA,HIGH);
//    digitalWrite(INC,LOW);
//    digitalWrite(INB,LOW);
//    Serial.println(left_output,4);


    

    calc_pub_odom(v_l,v_r,timegap);
    
    prev_right_wheel_tick_count = right_wheel_tick_count;
    prev_left_wheel_tick_count = left_wheel_tick_count;
    
    total_left_distance = total_left_distance + left_dis_gap;
    total_right_distance = total_right_distance + right_dis_gap;

    nh.spinOnce();
    //delay(1000);
  }
}


 
void calc_pub_odom(double v_l,double v_r,double timegap){

  double vel_trans=(v_l+v_r)/2;
  double anguler_vel=(v_l-v_r)/wheel_gap;
  
  double delta_yaw=anguler_vel*timegap/1000;
    
  yaw=yaw+delta_yaw;
  
  double velX=vel_trans*cos(yaw);
  double velY=vel_trans*sin(yaw);
  
  double delta_x=velX*timegap/1000;
  double delta_y=velY*timegap/1000;
  
  positionX=positionX+delta_x;
  positionY=positionY+delta_y;
  Serial.println(positionX,4);
  Serial.println(positionY,4);
  Serial.println("yaw=");
  Serial.println(yaw,4);
  
  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
    
  t.transform.translation.x = positionX;
  t.transform.translation.y = positionY;
    
  t.transform.rotation = tf::createQuaternionFromYaw(yaw);
  t.header.stamp = nh.now();
    
  broadcaster.sendTransform(t);
   
}

 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count++;  
    }    
  }
  else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    }
    else {
      right_wheel_tick_count--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count++;  
    }  
  }
  else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count--;  
    }   
  }
}
