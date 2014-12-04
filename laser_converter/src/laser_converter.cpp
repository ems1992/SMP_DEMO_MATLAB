#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#pragma GCC diagnostic pop

#include <boost/bind.hpp>

#include <string>

std::string output_frame_ = "odom";
laser_geometry::LaserProjection* projector_ = NULL;
tf::TransformListener* listener_ = NULL;
ros::Publisher* points_out_ = NULL;
ros::Subscriber* scans_in_ = NULL;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_->waitForTransform(
        scan_in->header.frame_id,
        output_frame_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
  {
     return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_->transformLaserScanToPointCloud(output_frame_, *scan_in, cloud,
    *listener_);

  points_out_->publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_converter");
  ros::NodeHandle nh;

  projector_ = new laser_geometry::LaserProjection();
  listener_ = new tf::TransformListener();

  nh.getParam("output_frame", output_frame_);

  points_out_ = new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("points_out", 1));
  scans_in_ = new ros::Subscriber(nh.subscribe<sensor_msgs::LaserScan>("scans_in", 1,
    scanCallback));

  ros::spin();
}
