/*  Copyright (C) 2019 by Bey Hao Yun <beyhy@artc.a-star.edu.sg>

    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
  OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#ifndef RVIP_RVIP_MAIN_H  // head guards
#define RVIP_RVIP_MAIN_H

#include <string>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/RegionOfInterest.h>



/*! \class RVIPobj
    \brief A Robotic Vision Integration Package (RVIP) class object.

    The RVIPobj class object conducts object localization functionalities using a 2D-3D hybrid
    approach. Within this class, functions defined with the cuboid_generation and pose_alignment
    library are utilized.
*/
class RVIPobj
{
  public:
    /*! \constructor Default constructor
    *
    *  Empty function. No class attributes initialized with default values.
    */
    RVIPobj(void);

    // Output topics
    // ros::Publisher pcl_pub;  // DEBUG - Not used
    // ros::Publisher scene_pcl_pub;  // DEBUG - Not used

    /*! \brief A ROS topic that outputs both position and orientation of detected object. */
    ros::Publisher marker_pub;

    int xmin;  /*!< \brief Minimum pixel x-coordinate value of the 2D bounding boxes */
    int xmax;  /*!< \brief Maximum pixel x-coordinate value of the 2D bounding boxes */
    int ymin;  /*!< \brief Minimum pixel y-coordinate value of the 2D bounding boxes */
    int ymax;  /*!< \brief Maximum pixel y-coordinate value of the 2D bounding boxes */

    std::string frame_id;  /*!< \brief The Fixed Frame value that will be read by RVIZ to be displayed. */

    float obj_x;  /*!< \brief Euclidean x value of object centroid. */
    float obj_y;  /*!< \brief Euclidean y value of object centroid. */
    float obj_z;  /*!< \brief Euclidean z value of object centroid. */
    float obj_height;  /*!< \brief Estimated height of object.*/
    float obj_width;   /*!< \brief Estimated width of object. */
    float obj_length;  /*!< \brief Estimated length of object.*/

    std::vector<tf::Transform> obj_tfs;  // Array of multiple transforms for detected objects.

    /*! \brief A tf::TransformBroadcaster object.
    *
    *    This tf::TransformBroadcaster will broadcast the final deduced tf::Transform, containing position and
    *    orientation object.
    *
    *    Refer to this link for more details: https://docs.ros.org/melodic/api/tf/html/c++/classtf_1_1TransformBroadcaster.html
    */
    tf::TransformBroadcaster br;  // TF Broadcaster of the deduced position and orientation of detected objects.

    pcl::PointCloud<pcl::PointXYZ> modelCloud;  /*!< \brief PointCloud cluster of generated estimated cuboid.*/
    sensor_msgs::PointCloud2 raw_pointcloud;  /*!< \brief PointCloud cluster of raw input PointCloud.*/
    std::vector<sensor_msgs::RegionOfInterest> boxes;  /*!< \brief A vector containing filtered sensor_msgs RegionOfInterest objects.*/

    visualization_msgs::MarkerArray obj_markers;  /*!< \brief An array of output visualization_msgs Markers*/

    /*! \brief A CallBack Function to receive PointCloud of scene.*/
    void pclCallBack(const sensor_msgs::PointCloud2ConstPtr& pcd);

    /*! \brief A CallBack Function to receive 2D Bounding Box pixel coordinates value of detected object in rectified image.*/
    void roiCallBack(const sensor_msgs::RegionOfInterestConstPtr& msg);

    /*! \brief A function that broadcast the output tf::Tranform and publish the output the visualization_msgs MarkerArray*/
    void visualizeDetectionOutput(void);

    /*! \brief Checks if an existing RegionOfInterest box in the boxes vector is within or a duplicate of the RegionOfInterest box that is to be added */
    bool isWithinBoundingBox(sensor_msgs::RegionOfInterest box_in_list, sensor_msgs::RegionOfInterest box_added);

    /*! \brief A function that runs Pose Estimation algorithm based on 2D Bounding Box and 3D PointCloud input*/
    void run(void);
};

#include "rvip_main.hpp"

#endif  // RVIP_RVIP_MAIN_H
