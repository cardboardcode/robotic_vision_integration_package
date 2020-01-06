/*  Copyright (C) 2019 by Bey Hao Yun <beyhy@artc.a-star.edu.sg>

    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
  OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#ifndef RVIP_CUBOID_GENERATION_H  // head guards
#define RVIP_CUBOID_GENERATION_H

#include <vector>
#include "rvip/rvip_main.h"
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>

// Includes for PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Includes for PCL Segmentation Tools
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

// Include for SAC Segmentation
#include <pcl/segmentation/sac_segmentation.h>

/*! \struct Point3f
    \brief A set of 3 float variables representing x, y and z coordinates of a 3D point.
*/
struct Point3f
{
        float x, y, z;
};

/*! \struct indices4i
    \brief A set of 4 integer variables representing the index of vertices that is used to indicate a plane of the generated cuboid.
*/
struct indices4i
{
        int p1, p2, p3, p4;
};

/*! \brief Calculates approximately the length(L), width(W) and height(H) of a detected object given the raw input PointCloud data. These estimations are achieved by dissecting the input PointCloud via 3 stages.
 *
 * The first stage extracts the ground plane PointCloud cluster.
 *
 * The second stage extracts the top plane of the object PointCloud cluster.
 *
 * The third stage extracts a minimally two-sided PointCloud cluster used for Pose Alignment.
*/
sensor_msgs::PointCloud2 calculateLWH(RVIPobj &object, sensor_msgs::PointCloud2 pcd);

/*! \brief Generates a cuboid PointCloud cluster based on estimated length, width and height. This cuboid is later used for Pose Alignment.
*/
void createCuboid(RVIPobj &object, float length, float width, float height);

/*! \brief Populates the cuboid PointCloud cluster with a uniform distribution of individual PointCloud point defined in 3-dimensions.
*/
void doUniformSampling(RVIPobj &object, std::vector<indices4i> planes,
                        std::vector<Point3f> normals, std::vector<Point3f> vertices);

/*! \brief Extracts the ground plane PointCloud cluster using VoxelGrid and Planar Segmentation PCL filters.
*/
void getGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

/*! \brief Extracts the top plane of the detected object PointCloud cluster using PassThrough, VoxelGrid, Statistical Outlier Removal and Planar Segmentation PCL filters.
*/
void getObjectTopPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointXYZ center, float std_dev);

/*! \brief Extracts a minimally two-sided PointCloud cluster used for Pose Alignment using PassThrough, VoxelGrid and Planar Segmentation.
*/
void getAlignPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointXYZ center, float std_dev);

#include "cuboid_generation.hpp"

#endif  // RVIP_CUBOID_GENERATION_H
