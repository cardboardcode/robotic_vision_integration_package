/*  Copyright (C) 2019 by Bey Hao Yun <beyhy@artc.a-star.edu.sg>

    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
  OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
// Include for gtest
#include <gtest/gtest.h>

// General Includes
#include <pcl/exceptions.h>

// Includes for ROS
#include "ros/ros.h"
#include <ros/package.h>

// Includes for Point Cloud Library and ROS PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// For Modularity and Debugging
#include "rvip/rvip_main.h"

TEST(RVIPobj_Functionality_TestSuite, test_isWithinBoundingBox)
{
  RVIPobj object;

  sensor_msgs::RegionOfInterest box_in_list;
  sensor_msgs::RegionOfInterest box_added;

  box_in_list.x_offset = 2;
  box_in_list.y_offset = 2;
  box_in_list.width = 2;
  box_in_list.height = 2;

  box_added.x_offset = 1;
  box_added.y_offset = 1;
  box_added.width = 2;
  box_added.height = 2;

  EXPECT_TRUE(object.isWithinBoundingBox(box_in_list, box_added));

  box_in_list.height = 3;

  EXPECT_FALSE(object.isWithinBoundingBox(box_in_list, box_added));
}

TEST(RVIPobj_Functionality_TestSuite, test_pclCallBack)
{
  std::string package_path = ros::package::getPath("rvip");
  std::string test_pcd_path = package_path + "/test/resrc/test_frame.pcd";

  RVIPobj object;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>
    (test_pcd_path, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file test_frame.pcd \n");
    EXPECT_EQ(1, 0);
  }

  sensor_msgs::PointCloud2 inputCloud;
  pcl::toROSMsg(*cloud, inputCloud);

  boost::shared_ptr<sensor_msgs::PointCloud2> pcd(new sensor_msgs::PointCloud2);

  *pcd = inputCloud;
  object.pclCallBack(pcd);
  int pcd_size = object.raw_pointcloud.width;

  EXPECT_GT(pcd_size, 0);
}

TEST(RVIPobj_Functionality_TestSuite, test_roiCallBack)
{
  RVIPobj object;
  boost::shared_ptr<sensor_msgs::RegionOfInterest> msg(new sensor_msgs::RegionOfInterest);

  msg->x_offset = 1;
  msg->y_offset = 1;
  msg->height = 1;
  msg->width = 1;
  object.roiCallBack(msg);

  int roi_boxes_size = object.boxes.size();

  EXPECT_EQ(roi_boxes_size, 0);

  msg->x_offset = 1;
  msg->y_offset = 1;
  msg->height = 71;
  msg->width = 71;
  object.roiCallBack(msg);
  roi_boxes_size = object.boxes.size();
  EXPECT_EQ(roi_boxes_size, 1);

  object.roiCallBack(msg);
  roi_boxes_size = object.boxes.size();
  EXPECT_EQ(roi_boxes_size, 1);

  msg->x_offset = 100;
  msg->y_offset = 100;
  msg->height = 71;
  msg->width = 71;
  object.roiCallBack(msg);
  roi_boxes_size = object.boxes.size();
  EXPECT_EQ(roi_boxes_size, 2);
}

TEST(RVIPobj_Functionality_TestSuite, test_run)
{

  std::string package_path = ros::package::getPath("rvip");
  std::string test_pcd_path = package_path + "/test/resrc/test_frame.pcd";

  RVIPobj object;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>
    (test_pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test_frame.pcd \n");
    EXPECT_EQ(1, 0);
  }
  sensor_msgs::PointCloud2 inputCloud;
  pcl::toROSMsg(*cloud, inputCloud);

  boost::shared_ptr<sensor_msgs::PointCloud2> pcd(new sensor_msgs::PointCloud2);

  *pcd = inputCloud;
  object.pclCallBack(pcd);
  int pcd_size = object.raw_pointcloud.width;
  EXPECT_GT(pcd_size, 0);

  boost::shared_ptr<sensor_msgs::RegionOfInterest> msg(new sensor_msgs::RegionOfInterest);

  msg->x_offset = 315;
  msg->y_offset = 507;
  msg->height = 127;
  msg->width = 128;
  object.roiCallBack(msg);

  EXPECT_EQ(object.boxes.size(), 1);

  ros::NodeHandle nh;
  object.marker_pub = nh.advertise<visualization_msgs::MarkerArray> ("/rvip/obj_marker", 100);

  object.run();

  EXPECT_GT(object.obj_tfs.size(), 0);

  cloud->clear();
  pcl::toROSMsg(*cloud, inputCloud);

  *pcd = inputCloud;
  object.pclCallBack(pcd);
  pcd_size = object.raw_pointcloud.width;
  EXPECT_EQ(pcd_size, 0);
  object.roiCallBack(msg);
  EXPECT_NO_THROW(object.run());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  ros::start();
  return RUN_ALL_TESTS();
}
