# Robotic Vision Integration Package (RVIP) Documentation

## What Is This?

**RVIP** is a ROS package that offers **easy integration of 2D-3D hybrid pose alignment processes** used in robotic vision applications.

By taking in both **2D RGB images** with **3D PointCloud** information, RVIP combines the high effieciency and accuracy of 2D computer vision architectures with the robust data points of depth information.

This is done so in an effort to ensure higher reliability and reduced volatility of object localization results, needed for safer robot-human collaboration spaces.

## How Does It Work?

The flow of data using RVIP is illustrated in the following diagram.

**Inputs**:
1. 3D PointCloud (sensor_msgs::PointCloud2)
2. 2D RGB Camera Image (sensor_msgs::Image)

**Outputs**:
1. Object Centroid Position, Orientation and Dimensions (visualisation_msgs::MarkerArray,tf::Transform)

![](rvip_architecture.jpg)
<center>RVIP Architecture Diagram</center>

### Stages
**[1]** - **3D PointCloud** and rectified **2D RGB Camera Image** are received from a 3D stereo-vision camera such as the Kinect ONE V2.

**[2]** - The rectified **2D RGB Camera Image** are given to an external ROS package that houses its own 2D Computer Vision Architecture. An example would be [darknet_ros](https://github.com/leggedrobotics/darknet_ros).

**[3]** - The role of RVIP starts here by receiving the synchronized pair of 3D PointCloud and the RegionOfInterest (RoI).
1. RVIP filters out valid RoI to avoid duplicates.
2. RVIP extracts the PointCloud cluster based on the given RoI information by segmenting the part of cluster that falls within the bounding box defined by the RoI.
3. RVIP extracts the lower larger plane of PointCloud cluster to determine the lower z-limit.
4. RVIP extracts the object's top plane to deter the upper z-limit.

**[4]** - RVIP estimates the length and width of the detected object using the object's top plane.
RVIP estimates the height by subtracting the upper and lower z-limits.

**[5]** - Based on the length, width and height of the object, a cuboid PointCloud is generated. A minimally 2-sided PointCloud is also extracted and used for the pose alignment later.

**[6]** - RVIP executes the pose alignment with [SUPER-4PCS](https://github.com/STORM-IRIT/OpenGR) as well as [Iterative Closest Point (ICP)](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php)

**[7]** - RVIP finally outputs the object centroid position (float x,y,z), object orientation (Quaternion) and its dimensions.

****

### Requirements
#### This section lists all third-party softwares used by **RVIP**.

**Click** the hyperlinks for download instructions.

1. [Ubuntu 18.04 (Bionic Beaver)](https://ubuntu.com/download/desktop)
    - For the operating system
2. [ROS Morenia Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - For the ROS base
3. OpenGR
    - For pose estimation. `Refer below for download instructions.`
4. [OpenCV 3.2.0](http://www.codebind.com/linux-tutorials/install-opencv-3-2-ubuntu-18-04-lts-linux/)
    - For reading and displaying images
5. [Point Cloud Library](https://askubuntu.com/questions/1134856/how-to-install-pcl-on-ubuntu-18-04)
    - For processing 3D information
6. darknet_ros
    - For object recognition using only 2D RGB images. `Refer below for download instructions.`


## Setup
This section provides instructions on how to install and use **RVIP**.

### There are **3** parts to the setup.
1. OpenGR Setup
2. RVIP Setup

# !**`WARNING`**!
> [WIP] The **OpenGR** library used in 0.0.1 alpha version is an outdated version. Please follow the instructions below to download the specific version of OpenGR highlighted.

### [OpenGR Setup]

1. **Download** the OpenGR library source codes from its GitHub repo.
> `$ git clone https://github.com/STORM-IRIT/OpenGR.git`

2. **Go** into the **OpenGR** directory
> `$ cd OpenGR`
3. **Git Revert** to the specified version of OpenGR.
> `$ git reset --hard 0967cd880950b35786b8fd098837c9eb1fe2aca4`

![](opengr_commit.png)
**Click** [here](https://github.com/STORM-IRIT/OpenGR/commit/0967cd880950b35786b8fd098837c9eb1fe2aca4) to go to the webpage with this commit.

4. **Create** and **Go** to a build directory
> `$ mkdir build && cd build`

5. **Build** the **OpenGR** using cmake.
> `$ cmake -DCMAKE_BUILD_TYPE=Release ..`

6. **Install** the **OpenGR** into your local workstation.
> `$ sudo make install`

### [RVIP Setup]

1. **Download** this repository via SSH into the `src` directory of your **catkin** workspace.
> `$ cd path/to/your/<workspace_name>/src`
>
> `$ git clone git@gitlab.com:cardboardcode/rvip_ros.git`

2. **Build** **RVIP** ROS package.
> `$ catkin build rvip -DCMAKE_BUILD_TYPE=Release`

3. **Source** your workspace.
> `$ source path/to/your/<workspace_name>/devel/setup.bash`



## Nodes
**RVIP** is represented by the `rvip` directory in this repository. It is a single ROS node.

#### Subscribed Topics

* `/camera_3d` ([sensor_msgs/PointCloud2])

    The PointCloud data of a scene captured by the camera.

* `/yolo_bounding_box_output` ([sensor_msgs/RegionOfInterest])

    The **RegionOfInterest** data (xmin, xmax, ymin, ymax, do_rectify) of detected objects outputed by **2D Computer Architectures**.

#### Published Topics

* `/rvip/model_pcl` ([pcl/PointCloud<pcl/PointXYZ>])

    Allows visualization of the generated cuboid PointCloud used for pose estimation.

* `/rvip/scene_pcl` ([pcl/PointCloud<pcl/PointXYZ])

    Allows visualization of the PointCloud of the captured scene.

* `/rvip/obj_marker` ([visualization_msgs::MarkerArray])

    Allows visualization of the generated cuboid containing information highlighted in the diagram above.

## Acknowledgement
Credits to **ROS-Industrial Consortium Asia Pacific** for providing the opportunity and industrial resources used in developing and testing this project.
