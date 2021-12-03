#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped P;
ros::Publisher tickpub("encoder_ticks", &P);

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks ther direction of rotation.
#define ENC_IN_LEFT_B 20
#define ENC_IN_RIGHT_B 21

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;


void setup() {

  nh.initNode();
  nh.advertise(tickpub);  // Open the serial port at 9600 bps
  Serial.begin(115200); 
 
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
 
   pub_ticks(left_wheel_tick_count,right_wheel_tick_count);
   nh.spinOnce();

}

void pub_ticks(int ltc,int rtc){

  Serial.println("LTC=");
  Serial.println(ltc);

  Serial.println("RTC=");
  Serial.println(rtc);

  P.header.stamp = nh.now();
  P.header.frame_id = "/encoder";

  P.vector.x=ltc;
  P.vector.y=rtc;
      
  tickpub.publish(&P);
   
}

 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) { //Forward
         
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count++;  
    }   
  }
  else { //Reverse
    
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
 
  if(val == HIGH) {// Forward
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count++;  
    }  
  }
  else { //Reverse
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count--;  
    }   
  }
   

}
