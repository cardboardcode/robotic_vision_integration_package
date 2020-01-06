/*  Copyright (C) 2019 by Bey Hao Yun <beyhy@artc.a-star.edu.sg>

    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
  OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#ifndef RVIP_POSE_ALIGNMENT_H  // head guards
#define RVIP_POSE_ALIGNMENT_H

#include "rvip/rvip_main.h"
#include "opengr/super4pcs.h"
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*! \brief Executes a 3-stage Pose Alignment and publishes a visualization_msgs Marker with a Transform of a detected objecct relative to the world and camera frame.
 *
 *  The first stage is a direct euclidean translation of cuboid to the object center.
 *  The second stage uses SUPER-4PCS to align cuboid and object.
 *  The third stage uses the IterativeClosestPoint algorithm to finetune the SUPER-4PCS alignment.
 */
void alignPose(RVIPobj &object, sensor_msgs::PointCloud2 pcd);

/*! \brief Checks if any of the detected object's 3 dimensions is smaller than 2cm.
 * If length, width or height of object is smaller than 2cm, return false.
 * Otherwise, return true.
 */
bool isValidObject(float obj_length, float obj_width, float obj_height);

/*! \brief Constructs a tf::Transform object based on deduced position and orientation of detected object.
 */
tf::Transform generateObjectTF(tf::Vector3 origin, tf::Quaternion tfqt);

/*! \brief Rotates the z-axis of object tf::Transform to align with world z-axis
 */
tf::Quaternion straightenObjectZAxis(tf::Vector3 origin, Eigen::Matrix4f final_transform);

#include "pose_alignment.hpp"

#endif  // RVIP_POSE_ALIGNMENT_H
