#include "pose_alignment.h"

using namespace std;

bool isValidObject(float obj_length, float obj_width, float obj_height){

  if (obj_length <= 0.02 || obj_width <= 0.02 || obj_height <= 0.02){
    return false;
  }
  return true;
}

// Rotate the z-axis of object tf to align with world z-axis
tf::Quaternion straightenObjectZAxis(tf::Vector3 origin, Eigen::Matrix4f final_transform){

  tf::Quaternion tfqt;
  tf::Quaternion q_rot;
  tf::Matrix3x3 object_orientation;
  tf::Vector3 obj_z_axis = tf::Vector3(static_cast<double>(final_transform(2,0)),
                                       static_cast<double>(final_transform(2,1)),
                                       static_cast<double>(final_transform(2,2)));
  tf::Vector3 world_z_axis = tf::Vector3(0, 0, -1);

  object_orientation.setValue(static_cast<double>(final_transform(0,0)), static_cast<double>(final_transform(0,1)), static_cast<double>(final_transform(0,2)),
                static_cast<double>(final_transform(1,0)), static_cast<double>(final_transform(1,1)), static_cast<double>(final_transform(1,2)),
                static_cast<double>(final_transform(2,0)), static_cast<double>(final_transform(2,1)), static_cast<double>(final_transform(2,2)));

  obj_z_axis = obj_z_axis.normalized();

  float numerator = obj_z_axis.dot(world_z_axis);
  float denominator = abs(obj_z_axis.length() * obj_z_axis.length());
  float angle = acos(numerator/denominator);

  tf::Vector3 rotate_axis = obj_z_axis.cross(world_z_axis);

  q_rot.setRotation(rotate_axis, -angle);

  object_orientation.getRotation(tfqt);
  if (!std::isnan(angle)){
    // ROS_INFO_STREAM("Angle is NAN. Rotation not applied.");
    tfqt = tfqt *  q_rot ;
  }
  tfqt.normalized();

  return tfqt;

}

tf::Transform generateObjectTF(tf::Vector3 origin, tf::Quaternion tfqt){

  tf::Transform objTF;

  objTF.setOrigin(origin);
  objTF.setRotation(tfqt);

  return objTF;

}

void alignPose(RVIPobj &object, sensor_msgs::PointCloud2 pcd)
{

        // DEBUG - Initiate timestamp for performance checking
        // ros::WallTime start_, end_;
        // start_ = ros::WallTime::now();

        // sceneCloud refers to the pcl of the top plane of the object
        pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(pcd, *sceneCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr localModelCloud(new pcl::PointCloud<pcl::PointXYZ>);
        *localModelCloud = object.modelCloud;

        // Check if sceneCloud and localModelCloud is empty
        // If empty, exit function. Otherwise, continue alignment.
        if (sceneCloud->points.size() == 0 || localModelCloud->points.size() == 0) {
                ROS_WARN_STREAM("sceneCloud or localModelCloud is empty. Unable to align empty clouds.");
                return;
        }

        if (!isValidObject(object.obj_length, object.obj_width, object.obj_height)){
              ROS_WARN("Invalid cuboid generated.\n");
              return;
        }
        // alignCloud refers to the pcl specifically used for all align functions in the 3 Stage Alignment
        pcl::PointCloud<pcl::PointXYZ>::Ptr alignCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // finalTransformedCloud refers to the pcl that captures the final transformed pcl of localModelCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr finalTransformedCloud (new pcl::PointCloud<pcl::PointXYZ> ());
        finalTransformedCloud->header.frame_id = object.frame_id;

        // Initializing variables needed for pose alignment.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::Super4PCS<pcl::PointXYZ, pcl::PointXYZ> super4pcs;
        Eigen::Matrix4f super4pcs_transform, icp_transform, final_transform;

        // Transform happen in 3 stages:
        // Stage 1: Direct translation to the object center.
        Eigen::Matrix4f translate_to_obj_center = Eigen::Matrix4f::Identity();
        translate_to_obj_center(0,3) = object.obj_x;
        translate_to_obj_center(1,3) = object.obj_y;
        translate_to_obj_center(2,3) = object.obj_z;
        pcl::transformPointCloud (*localModelCloud, *localModelCloud, translate_to_obj_center);

        try{
                // Stage 2: Using SUPER-4PCS to align using global registration
                // Setting SUPER-4PCS parameters
                super4pcs.options_.delta = 0.01;
                super4pcs.options_.configureOverlap(0.2);
                super4pcs.options_.max_time_seconds = 1;
                super4pcs.options_.sample_size = 100;
                super4pcs.options_.max_translation_distance = -1;
                super4pcs.setInputSource(sceneCloud);
                super4pcs.setInputTarget(localModelCloud);
                super4pcs.align(*alignCloud);
                // final cloud is the input source which has been transformed
                super4pcs_transform = super4pcs.getFinalTransformation().inverse();
                pcl::transformPointCloud (*localModelCloud, *localModelCloud, super4pcs_transform);

                // Stage 3: Using ICP for fine-tuning of position
                icp.setInputSource(sceneCloud);
                icp.setInputTarget(localModelCloud);
                // Setting ICP parameters
                icp.setEuclideanFitnessEpsilon (0.000001);
                icp.align(*alignCloud);
                // Inverting transform to get tf frame from world origin to object position
                icp_transform = icp.getFinalTransformation().inverse();
                pcl::transformPointCloud (*localModelCloud, *finalTransformedCloud, icp_transform);

                // Compiling all transforms made.
                final_transform = icp_transform * super4pcs_transform * translate_to_obj_center;

        }
        catch(pcl:: PCLException& e) {
                ROS_WARN("Unable to align. SUPER-4PCS and/or ICP failed.\n");
                return;
        }

        static tf::TransformBroadcaster br;
        tf::Vector3 origin2;
        origin2.setValue(object.obj_x, object.obj_y, object.obj_z);
        tf::Quaternion tfqt2 = straightenObjectZAxis(origin2, final_transform);

        tf::Transform objTF2 = generateObjectTF(origin2, tfqt2);

        br.sendTransform(tf::StampedTransform(objTF2, ros::Time::now(), object.frame_id, "T-T"));

          tf::Vector3 origin;
          origin.setValue(object.obj_x, object.obj_y, object.obj_z);

          tf::Quaternion tfqt = straightenObjectZAxis(origin, final_transform);
          tf::Transform objTF = generateObjectTF(origin, tfqt);

          object.obj_tfs.push_back(objTF);
          // DEBUG - Show marker of object with dynamic size, position and orientation.
          visualization_msgs::Marker marker;
          marker.header.frame_id = object.frame_id;
          marker.header.stamp = ros::Time();
          marker.type = visualization_msgs::Marker::CUBE;

          marker.id = object.obj_markers.markers.size();
          marker.pose.position.x = object.obj_x;
          marker.pose.position.y = object.obj_y;
          marker.pose.position.z = object.obj_z;
          marker.pose.orientation.x = tfqt.x();
          marker.pose.orientation.y = tfqt.y();
          marker.pose.orientation.z = tfqt.z();
          marker.pose.orientation.w = tfqt.w();
          marker.scale.x = object.obj_length;
          marker.scale.y = object.obj_width;
          marker.scale.z = object.obj_height;
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;
          marker.lifetime = ros::Duration(5.0);

          object.obj_markers.markers.push_back(marker);

          // DEBUG - Timestamp END - Print out time taken for pose alignment.
          // end_ = ros::WallTime::now();
          // double execution_time = (end_ - start_).toNSec() * 1e-6;
          // ROS_INFO("Execution time for [Pose Alignment] (ms):  %f                  \r", execution_time);

}
