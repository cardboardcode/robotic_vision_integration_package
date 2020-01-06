// General Includes
#include <math.h>
#include <cstdlib>
#include <sstream>

// Includes for ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

// Includes for ROS Image
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>

// Include for darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

ros::Publisher roi_pub;

void roiCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  darknet_ros_msgs::BoundingBox input;
  sensor_msgs::RegionOfInterest output;

  for (int i = 0; i < msg->bounding_boxes.size(); i++){
      input = msg->bounding_boxes[i];

      output.x_offset = input.xmin;
      output.y_offset = input.ymin;
      output.width = input.xmax - input.xmin;
      output.height = input.ymax - input.ymin;

      roi_pub.publish(output);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rvip_roi_parser");

  ros::NodeHandle n;

  // Subscription to custom YOLO bounding boxes
  ros::Subscriber yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, roiCallback);

  // Publication of generic sensor_msgs RegionOfInterest message
  roi_pub = n.advertise<sensor_msgs::RegionOfInterest>("/rvip_roi_parser/roi", 1000);

  ros::spin();
  return 0;
}
