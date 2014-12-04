/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 *
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#define SCALE 180

ros::NodeHandle  nh;

Servo servo;
char base_link[] = "/servo";
char odom[] = "/odom";

void servo_cb( const sensor_msgs::Joy& cmd_msg){
  uint16_t value = (cmd_msg.axes[0]*SCALE)+90;
  servo.write(value); //set servo angle, should be from 0-180
  float angle = servo.read();
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo.attach(9); //attach it to pin 9
}
//seems to be required
void loop()
{
  nh.spinOnce();
}
