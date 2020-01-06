#include "cuboid_generation.h"

using namespace std;

// Creates a point cloud of pseudo STL vertices
void doUniformSampling(RVIPobj &object, std::vector<indices4i> planes, std::vector<Point3f> normals, std::vector<Point3f> vertices)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        new_cloud->header.frame_id = object.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), new_cloud->header.stamp);
        new_cloud->is_dense = object.modelCloud.is_dense;
        new_cloud->width = object.modelCloud.width;
        new_cloud->height = object.modelCloud.height;
        new_cloud->points.resize(new_cloud->width * new_cloud->height);

        // Populating the pointcloud with the initial corner vertices.
        for (int i = 0; i < vertices.size(); i++ ) {
                object.modelCloud.points.push_back( pcl::PointXYZ( vertices.at(i).x,
                                                             vertices.at(i).y, vertices.at(i).z ) );
        }

        for (int i = 0; i < planes.size(); i++) {
                // Determine the planar axis and its dimensions upper and lower bound.
                float k = 0;
                float m = 0;
                float old_m = 0;
                float leaf_size = 0.01;
                std::vector <int> planar_axis;

                std::vector<float> upper_bounds;
                std::vector<float> lower_bounds;
                upper_bounds.resize(3);
                lower_bounds.resize(3);

                upper_bounds.at(0) = min(  min(vertices.at(planes.at(i).p1).x, vertices.at(planes.at(i).p2).x), min(vertices.at(planes.at(i).p3).x, vertices.at(planes.at(i).p4).x)  );
                lower_bounds.at(0) = max(  max(vertices.at(planes.at(i).p1).x, vertices.at(planes.at(i).p2).x), max(vertices.at(planes.at(i).p3).x, vertices.at(planes.at(i).p4).x)  );

                upper_bounds.at(1) = min(  min(vertices.at(planes.at(i).p1).y, vertices.at(planes.at(i).p2).y), min(vertices.at(planes.at(i).p3).y, vertices.at(planes.at(i).p4).y)  );
                lower_bounds.at(1) = max(  max(vertices.at(planes.at(i).p1).y, vertices.at(planes.at(i).p2).y), max(vertices.at(planes.at(i).p3).y, vertices.at(planes.at(i).p4).y)  );

                upper_bounds.at(2) = min(  min(vertices.at(planes.at(i).p1).z, vertices.at(planes.at(i).p2).z), min(vertices.at(planes.at(i).p3).z, vertices.at(planes.at(i).p4).z)  );
                lower_bounds.at(2) =  max(  max(vertices.at(planes.at(i).p1).z, vertices.at(planes.at(i).p2).z), max(vertices.at(planes.at(i).p3).z, vertices.at(planes.at(i).p4).z)  );

                // planar_axis should only ever have two elements.
                if (normals.at(i).x == 0 )
                        planar_axis.push_back(0);

                if (normals.at(i).y == 0)
                        planar_axis.push_back(1);

                if (normals.at(i).z == 0)
                        planar_axis.push_back(2);

                k = upper_bounds.at(planar_axis.at(0));
                m = old_m = upper_bounds.at(planar_axis.at(1));

                // Populate point cloud with uniform sampling of planar mesh
                while (k <= lower_bounds.at( planar_axis.at(0) )) {

                        m = old_m;

                        upper_bounds.at(planar_axis.at(1)) = old_m;

                        while (m <= lower_bounds.at( planar_axis.at(1) )) {
                                bool isCorner = false;
                                std::vector<float> new_point;

                                new_point.push_back(
                                        (abs(normals.at(i).x)) ?
                                        vertices.at(planes.at(i).p1).x : upper_bounds.at(0));
                                new_point.push_back(
                                        (abs(normals.at(i).y)) ?
                                        vertices.at(planes.at(i).p1).y : upper_bounds.at(1));
                                new_point.push_back(
                                        (abs(normals.at(i).z)) ?
                                        vertices.at(planes.at(i).p1).z : upper_bounds.at(2));

                                // Iterate through the corner vertices and detect if the new point added is a corner vertex.
                                for (int temp = 0; temp < vertices.size(); temp++ ) {
                                        // if corner vertex detected, set boolean flag to true.
                                        if (new_point.at(0)== vertices.at(temp).x && new_point.at(1)== vertices.at(temp).y && new_point.at(2)== vertices.at(temp).z) {
                                                isCorner = true;
                                        }

                                }

                                // If not a corner, add the 3d point to the pointcloud
                                if (!isCorner) {
                                        object.modelCloud.points.push_back( pcl::PointXYZ( new_point.at(0),
                                                                                     new_point.at(1), new_point.at(2) ) );
                                }

                                m += leaf_size;
                                upper_bounds.at(planar_axis.at(1)) += leaf_size;
                        }
                        k += leaf_size;
                        upper_bounds.at(planar_axis.at(0)) += leaf_size;
                }
        }
}

void createCuboid(RVIPobj &object, float length, float width, float height)
{

        std::vector<Point3f> normals;
        std::vector<Point3f> vertices;
        std::vector<indices4i> planes;

        float x = length/2;
        float y = width/2;
        float z = height/2;

        // Populating hard-coded normal vectors of cuboid in vertices array
        Point3f temp;
        temp.x = 1.0; temp.y = 0.0; temp.z = 0.0; // Front face
        normals.push_back(temp);
        temp.x = 0.0; temp.y = 1.0; temp.z = 0.0; // Left face
        normals.push_back(temp);
        temp.x = 0.0; temp.y = 0.0; temp.z = 1.0; // Top face
        normals.push_back(temp);

        // Populating hard-coded vertices of cuboid in vertices array
        temp.x = x; temp.y = -y; temp.z = -z;
        vertices.push_back(temp);
        temp.x = x; temp.y = y; temp.z = -z;
        vertices.push_back(temp);
        temp.x = x; temp.y = y; temp.z = z;
        vertices.push_back(temp);
        temp.x = x; temp.y = -y; temp.z = z;
        vertices.push_back(temp);
        temp.x = -x; temp.y = -y; temp.z = -z;
        vertices.push_back(temp);
        temp.x = -x; temp.y = y; temp.z = -z;
        vertices.push_back(temp);
        temp.x = -x; temp.y = y; temp.z = z;
        vertices.push_back(temp);
        temp.x = -x; temp.y = -y; temp.z = z;
        vertices.push_back(temp);

        // Populating the planes array with the corresponding indices of coplanar vertices
        indices4i temp_plane;
        temp_plane.p1 = 0; temp_plane.p2 = 1; temp_plane.p3 = 2; temp_plane.p4 = 3; // Front face
        planes.push_back(temp_plane);

        temp_plane.p1 = 1; temp_plane.p2 = 5; temp_plane.p3 = 6; temp_plane.p4 = 2; // Left face
        planes.push_back(temp_plane);

        temp_plane.p1 = 3; temp_plane.p2 = 2; temp_plane.p3 = 6; temp_plane.p4 = 7; // Top face
        planes.push_back(temp_plane);

        doUniformSampling(object, planes, normals, vertices);
}

void getGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  // Remove all NAN points from input cloud and avoid error when setting parameters for filter.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *input, indices);

  double leafsize = 0.005;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (leafsize, leafsize, leafsize);
  sor.filter (*input);

  // Create required parameters to be passed into standard PCL library functions for planar segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (input);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*input);
}

void getObjectTopPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointXYZ center, float std_dev)
{
  std::string filter_fieldname(1,'z');
  const std::string* ptr = &filter_fieldname;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input);
  pass.setFilterFieldName (*ptr);
  pass.setFilterLimits(0.0,center.z - std_dev);
  pass.setFilterLimitsNegative(false);
  pass.filter (*input);

  // Remove all NAN points from input cloud and avoid error when setting parameters for filter.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *input, indices);

  double leafsize = 0.001;
  pcl::VoxelGrid<pcl::PointXYZ> vor;
  vor.setInputCloud (input);
  vor.setLeafSize (leafsize, leafsize, leafsize);
  vor.filter (*input);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input);
  sor.setMeanK(20);
  sor.setStddevMulThresh(0.002);
  sor.filter(*input);

  // Create required parameters to be passed into standard PCL library functions for planar segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.002);
  seg.setInputCloud (input);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*input);
}

void getAlignPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointXYZ center, float std_dev)
{
  std::string filter_fieldname(1,'z');
  const std::string* ptr = &filter_fieldname;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input);
  pass.setFilterFieldName (*ptr);
  pass.setFilterLimits(0.0,center.z - std_dev);
  pass.setFilterLimitsNegative(false);
  pass.filter (*input);

  // Remove all NAN points from input cloud and avoid error when setting parameters for filter.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *input, indices);

  double leafsize = 0.005;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (leafsize, leafsize, leafsize);
  sor.filter (*input);

  // Create required parameters to be passed into standard PCL library functions for planar segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);
  seg.setInputCloud (input);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*input);
}

// Estimates the length, width and height of the detected object.
sensor_msgs::PointCloud2 calculateLWH(RVIPobj &object, sensor_msgs::PointCloud2 pcd)
{
        // DEBUG - Initiate timestamp for performance checking
        // ros::WallTime start_, end_;
        // start_ = ros::WallTime::now();

        sensor_msgs::PointCloud2 temp;
        // Preparing Pointcloud for processing
        auto yoloPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(pcd, *pointCloud);
        yoloPointCloud->header.frame_id = pointCloud->header.frame_id;
        yoloPointCloud->header.stamp = pointCloud->header.stamp;
        yoloPointCloud->is_dense = pointCloud->is_dense;
        yoloPointCloud->width = pointCloud->width;
        yoloPointCloud->height = pointCloud->height;
        yoloPointCloud->points.resize(yoloPointCloud->width * yoloPointCloud->height);

        // Filter the point cloud to derive whichever points that is in the YOLO bounding box.
        for (int y =object.ymin; y < object.ymax; y++) {
                for (int x=object.xmin; x < object.xmax; x++) {

                        auto& point = yoloPointCloud->at(x,y);
                        point.x = pointCloud->at(x, y).x;
                        point.y = pointCloud->at(x, y).y;
                        point.z = pointCloud->at(x, y).z;
                }
        }

        // Assigning input to point to a copy of the scene point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud (new pcl::PointCloud<pcl::PointXYZ>);
        *groundCloud = *pointCloud;

        getGroundPlane(groundCloud);

        //Determining the centroid of the ground plane
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointXYZ center;
        for (int i = 0; i < groundCloud->size(); i++) {
                centroid.add (groundCloud->at(i));
        }
        centroid.get(center);

        // Calculating standard deviation of the ground plane z value to extract as best a planar segment as possible.
        float std_dev = 0.0;
        for (int i = 0; i < groundCloud->size(); i++) {
                std_dev += pow(groundCloud->at(i).z - center.z, 2);
        }
        std_dev = sqrt(std_dev / groundCloud->size());

        // Extracting the top plane of the object detected.
        // Passthrough, VoxelGrid, Statistical Outlier Removal, Planar Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr objplaneCloud (new pcl::PointCloud<pcl::PointXYZ>);
        *objplaneCloud = *yoloPointCloud;

        getObjectTopPlane(objplaneCloud,center,std_dev);

        if (objplaneCloud->points.size() == 0) {
                ROS_WARN_STREAM("objplaneCloud size is 0 ");
                return temp;
        }

        //Determining the centroid of the object plane
        pcl::CentroidPoint<pcl::PointXYZ> plane_centroid;
        pcl::PointXYZ plane_pos;
        for (int i = 0; i < objplaneCloud->size(); i++) {
                plane_centroid.add (objplaneCloud->at(i));
        }
        plane_centroid.get(plane_pos);

        // Derive the length and width of the object plane.
        std::vector<cv::Point2f> pts;
        cv::Point2f curr_pt(0,0);
        for (int i = 0; i < objplaneCloud->points.size (); i++) {
                curr_pt.x = objplaneCloud->points[i].x;
                curr_pt.y = objplaneCloud->points[i].y;
                pts.push_back(curr_pt);
        }

        cv::RotatedRect minRect;
        try{
                minRect = minAreaRect(pts);
        }
        catch (cv::Exception& e) {
                ROS_WARN_STREAM("minAreaRect func failed.");
                return temp;
        }

        object.obj_width = minRect.size.width;
        object.obj_length =  minRect.size.height;
        object.obj_height = abs(center.z - plane_pos.z);

        // DEBUG - Check the length, width and height of object detected.
        // ROS_INFO_STREAM("[In cuboid_generation.hpp]");
        // ROS_INFO_STREAM("[width = " << minRect.size.width << " ,length = " << minRect.size.height << " ,height =  " << abs(center.z - plane_pos.z) << "]" );

        createCuboid(object, minRect.size.height, minRect.size.width, abs(center.z - plane_pos.z));

        // Conducting manual passthrough filter on the yolo-box filtered pcl
        // Passthrough, VoxelGrid, Planar Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr alignCloud (new pcl::PointCloud<pcl::PointXYZ>);
        *alignCloud = *yoloPointCloud;
        // Based on the assumption that two planes of the object is detected by a depth sensor positioned at a 45 degrees angle.

        getAlignPointCloud(alignCloud,center,std_dev);

        // Update global variables of object center later used for pose alignment.
        object.obj_x = plane_pos.x;
        object.obj_y = plane_pos.y;
        object.obj_z = plane_pos.z + abs(center.z - plane_pos.z)/2;

        sensor_msgs::PointCloud2 outputCloud;
        pcl::toROSMsg(*alignCloud, outputCloud);

        // DEBUG - Timestamp END - Print out time taken for pose alignment.
        // end_ = ros::WallTime::now();
        // double execution_time = (end_ - start_).toNSec() * 1e-6;
        // ROS_INFO("Execution time for [Cuboid Generation] (ms):  %f                  \r", execution_time);
        // fflush(stdout);

        return outputCloud;
}
