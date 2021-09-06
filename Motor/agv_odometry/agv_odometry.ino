#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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
const double wheel_radius = 0.0685;//in m
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
int interval = 1500;
long previousMillis = 0;
long currentMillis = 0;
long timegap = 0;
 
void setup() {

  nh.initNode();
  broadcaster.init(nh);
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
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

    calc_pub_odom(v_l,v_r,timegap);
    
    prev_right_wheel_tick_count = right_wheel_tick_count;
    prev_left_wheel_tick_count = left_wheel_tick_count;
    
    total_left_distance = total_left_distance + left_dis_gap;
    total_right_distance = total_right_distance + right_dis_gap;
    
    Serial.println("Distance: ");
    Serial.println(total_left_distance,3);
    Serial.println(total_right_distance,3);
    //Serial.println(timegap);
    Serial.println("v_l =");
    Serial.println(v_l,3);
    Serial.println("v_r =");
    Serial.println(v_r,3);        
    
    //Serial.println(2*3.14*0.05*(right_wheel_tick_count/438));
    //Serial.println(2*3.14*0.05*(left_wheel_tick_count/438));
    Serial.println();
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

  
  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
    
  t.transform.translation.x = positionX;
  t.transform.translation.y = positionY;
    
  t.transform.rotation = tf::createQuaternionFromYaw(yaw);
  t.header.stamp = nh.now();
    
  broadcaster.sendTransform(t);
  nh.spinOnce();
   
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
