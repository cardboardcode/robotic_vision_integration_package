#include "rvip_main.h"
#include "rvip/cuboid_generation.h"
#include "rvip/pose_alignment.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

RVIPobj::RVIPobj(void){}

// If "box in list" is within "the box to be added", return true.
// Otherwise, return false.
bool RVIPobj::isWithinBoundingBox(sensor_msgs::RegionOfInterest box_in_list, sensor_msgs::RegionOfInterest box_added ){

    if (box_in_list.x_offset >= box_added.x_offset
    &&  box_in_list.y_offset >= box_added.y_offset
    &&  box_in_list.width <= box_added.width
    &&  box_in_list.height <= box_added.height)
    {
      return true;
    }
    else
    {
      return false;
    }

}

void RVIPobj::pclCallBack(const sensor_msgs::PointCloud2ConstPtr& pcd){

  raw_pointcloud = *pcd;

}

void RVIPobj::roiCallBack(const sensor_msgs::RegionOfInterestConstPtr& msg){

  sensor_msgs::RegionOfInterest input;

    input = *msg;

    int x_diff = input.width;
    int y_diff = input.height;

    // Reject bounding box if it is less than 70 pixels wide.
    // Such bounding boxes are certain to produce invalid cuboid.
    if (x_diff <= 70 || y_diff <=70){
      return;
    }

    // If boxes vector array is empty, assign and skip the latter for loop evaluation.
    if (boxes.size() == 0){
        boxes.push_back(input);
        return;
    }
    // Filter out any bounding boxes which are within a larger bounding box.
    // Iterate through boxes vector array and compare with each box in list.
    for (int j = 0; j < boxes.size(); j++){
      // If the box in the list is encapsulated by the box to be added,
      // Replace the box in the list with the box to be added.
      // Otherwise, ignore the box to be added.
      if (isWithinBoundingBox(boxes.at(j),input)){
         boxes.erase(boxes.begin()+j);
         boxes.insert(boxes.begin()+j, input);
      }
      else{
         boxes.push_back(input);
         break;
      }
    }

}

void RVIPobj::visualizeDetectionOutput(void){
  // For visualization purposes only.
  // Broadcast localization of different objects with visualization markers.
  for (int i = 0; i < obj_tfs.size(); i++){
    br.sendTransform(tf::StampedTransform(obj_tfs.at(i), ros::Time::now(), frame_id, "object_" + std::to_string(i)));
  }
  // DEBUG
  marker_pub.publish(obj_markers);
}

void RVIPobj::run(void){

  obj_tfs.clear();
  obj_markers.markers.clear();

  int no_of_objs_detected = boxes.size();

  // If no objects detected, skip iteration.
  if(no_of_objs_detected == 0){
      return;
  }

  for (int i = 0; i < no_of_objs_detected; i++){
      sensor_msgs::PointCloud2 yolo_pcl;

      xmin = boxes.at(i).x_offset;
      xmax = boxes.at(i).x_offset + boxes.at(i).width;
      ymin = boxes.at(i).y_offset;
      ymax = boxes.at(i).y_offset + boxes.at(i).height;

      try{
          // Takes in the raw_pointcloud and outputing the following:
          // 1. The top plane of the object.
          // 2. Length (L), Width (W), Height (H) of the detected object.
          // 3. A cuboid point cloud that will be used to localize the object.
          yolo_pcl = calculateLWH(*this, raw_pointcloud);

          // Returns a labelled tf frame and a grasp validation message.
          alignPose(*this, yolo_pcl);
      }
      catch(pcl:: PCLException& e) {
          ROS_WARN_STREAM("[pcl::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!\n");
          ROS_WARN_STREAM("Exception caught. Exiting safe.");
          return;
      }

      }

      // Only when objects has been detected and localized, visualization of said objects is executed.
      if (obj_tfs.size() > 0){
          visualizeDetectionOutput();
      }

      boxes.clear();

}
