#include "BMSerial.h"
#include "RoboClaw.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

//Roboclaw Address
#define address 0x80

//Definte terminal for display. Use hardware serial pins 0 and 1
//BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(15,14,10000);

ros::NodeHandle nh;
int init_Speed_val = 10; 
int i = 1;
int RPM_val = 30;
int Speed_val = map(RPM_val,0,80,0,127);

void process_command(const std_msgs::Int32& command)
{
  switch(command.data)
  {
    case 1:
      if(i == 1)
      {
        roboclaw.ForwardM1(address, init_Speed_val);
        delay(50);
      }
    break;
    case 2:
      if(i == 1)
      {
        roboclaw.BackwardM1(address,init_Speed_val);
        delay(50);
      }
    break;
    case 3:
      roboclaw.ForwardM1(address,0);
      delay(50);
      i=1;
    break;
    case 4:
    if(i == 1)
    {
      roboclaw.ResetEncoders(address);
      i=2;
    }
    break;
  }
}

std_msgs::Int32 str_msg;
std_msgs::Int32 com_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::Int32> command("command", &process_command);


void setup() {
  //Open terminal and roboclaw at 38400bps
  //terminal.begin(57600); 
  roboclaw.begin(38400);
  //Use Quadrature Encoder and Enable encoders in non-RC mode
  roboclaw.SetM1EncoderMode(address,0x00); //0x00 = 00000000 --> first bit is 0 for Quadrature, 1 for Analog; 8th bit enables/disables the RC mode  

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(command);
}

//int serialData = 0;

int enc_val = 0;
int deg_val = 0;
int j = 1;

void loop() {
  //Zero the motor
  deg_val = map(enc_val,-4200,4200,-180,180);
  /*
  serialData = terminal.read();
  if (i==1 && serialData==49)
  {
    roboclaw.ForwardM1(address,init_Speed_val);
    delay(50);
  }
  else if (i==1 && serialData==50)
  {
    roboclaw.BackwardM1(address,init_Speed_val);
    delay(50);
  }
  else if (serialData==51)
  {
    roboclaw.ForwardM1(address,0);
    delay(50);
    i=1;
  }
  else if(serialData==52)
  {
    roboclaw.ResetEncoders(address);
    i=2;
  }
  */
  if (i==2 && enc_val < 4200)
  {
    roboclaw.ForwardM1(address,Speed_val);
    enc_val = roboclaw.ReadEncM1(address,0,0);
    //terminal.println(deg_val);
    str_msg.data = deg_val;
    chatter.publish( &str_msg );
    delay(5);
  }
  if (i==2 && enc_val >= 4200)
  {
    roboclaw.ForwardM1(address,0);
    i = 3;
    delay(5);
  }
  if (i==3 && enc_val > -4200)
  {
    roboclaw.BackwardM1(address,Speed_val);
    enc_val = roboclaw.ReadEncM1(address,0,0);
    //terminal.println(deg_val);
    str_msg.data = deg_val;
    chatter.publish( &str_msg );
    delay(5);
  }
  if (i==3 && enc_val <= -4200)
  {
    roboclaw.BackwardM1(address,0);
    i = 2;
    delay(5);
  }
  nh.spinOnce();
}


