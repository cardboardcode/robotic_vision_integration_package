/*  Copyright (C) 2019 by Bey Hao Yun <beyhy@artc.a-star.edu.sg>

    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
  OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
// General Includes
#include <pcl/exceptions.h>

// Includes for ROS
#include "ros/ros.h"

// Includes for Point Cloud Library and ROS PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// For Modularity and Debugging
#include "rvip/rvip_main.h"

int main(int argc, char **argv)
{
        // Initializing ROS node
        ros::init(argc, argv, "rvip_0.0.2");
        ros::NodeHandle n;

        // Initializing RVIP object.
        RVIPobj object;

        // Getting frame_id from roslaunch parameter input.
        n.getParam("/rvip_package/frame_id",   object.frame_id);

        // Subscribing to a cropped pcl
        ros::Subscriber pcl_sub = n.subscribe("camera_3d", 1, &RVIPobj::pclCallBack, &object);
        // Subscription to custom ROI bounding boxes
        ros::Subscriber roi_sub = n.subscribe("rvip_roi_parser/roi", 1, &RVIPobj::roiCallBack, &object);

        // DEBUG Publishing PointCloud of generated cuboid
        // object.pcl_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("/rvip/model_pcl", 1000);
        // DEBUG Publishing PointCloud of dissected scene
        // object.scene_pcl_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("/rvip/scene_pcl", 1000);
        // DEBUG - Publishing marker of object detected and transformed.
        object.marker_pub = n.advertise<visualization_msgs::MarkerArray> ("/rvip/obj_marker", 100);

        // Run RVIP main module
        while (ros::ok())
        {
            // Spinning once to update input ROI and PointCloud data.
            ros::spinOnce();
            // Processing and publishing results using marker_pub.
            object.run();
        }

        return 0;
}
