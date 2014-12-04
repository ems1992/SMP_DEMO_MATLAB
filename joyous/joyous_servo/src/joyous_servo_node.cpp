#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//super-crappy bridge between an arduino-controlled servo and ROS
//Jack Newton, newtonj@vt.edu

const static std::string base_link = "/servo";
const static std::string odom = "/odom";
const static std::string cam = "/cam";
tf::StampedTransform trans;

void servo_callback(const std_msgs::Int32::ConstPtr& angle_msg)
{
    ROS_INFO("angle: %d", angle_msg->data);
    tf::Quaternion quat = tf::createQuaternionFromRPY(((angle_msg->data+90.0)/180.0)*M_PI, 0.0, 0.0);

    trans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    trans.frame_id_ = odom;
    trans.child_frame_id_ = base_link;
    trans.child_frame_id_ = cam;
    trans.setRotation(quat);
    trans.stamp_ = ros::Time::now(); //probably not valid, need to stamp the angle on the arduino itself
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_servo_bridge");

    ros::NodeHandle nh;

    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Int32>("servo_angle", 10, servo_callback);
    ros::Rate loop_rate(10);
    tf::TransformBroadcaster broadcaster;

    ROS_INFO("Beginning broadcast.");

    while(ros::ok())
    {
        broadcaster.sendTransform(trans);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
