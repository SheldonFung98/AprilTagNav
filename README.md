# AprilTagNav ROS
## AprilTag QR Code Localization Module for ROS

This project provides RGB image localization functionality based on AprilTag, suitable for scenarios where a cleaning robot needs to return to its charging station. Additionally, this project includes a stop function tailored for this scenario.


## dependency

* ira_laser_tools

## Usage
### Compile
```
mkdir -p catkin_ws/src && cd catkin_ws/src
https://github.com/SheldonFung98/AprilTagNav.git
cd ..
catkin_make_isolated
source devel/setup.sh
```
### Run
```
roslaunch apriltag_ros detect.launch
```
## Code Explain
This project relies on apriltag and apriltag_ros. Their code repositories are as follows:
```
https://github.com/AprilRobotics/apriltag.git
https://github.com/AprilRobotics/apriltag_ros.git
```

### QR Code Localization Functionality
* pose_publisher_trig.cpp
1. By subscribing to the topic /tag_detections in apriltag_ros, we can obtain the coordinates and orientation (quaternion) of the QR code in the camera coordinate system.
2. After obtaining the roll, pitch, and yaw from the quaternion, considering only one dimension, we use trigonometric relationships to transform the data from the camera coordinate system to the QR code coordinate system.

### Obstacle Detection Functionality
* Obstacle Input

apriltagnav/apriltag_ros/launch/detect.launch
```
<launch>
    <node pkg="ira_laser_tools" name="laserscan_multi_merger" type="laserscan_multi_merger" output="screen">
        ...
        <param name="laserscan_topics" value ="/scan /rgbd_pc_id1/scan" />
    </node>
    ...
</launch>
```

Obstacles can simultaneously receive data from the laser scanner (LaserScan) and RGB-D point cloud (PointCloud). This functionality subscribes to the topics `/scan` and `/rgbd_pc_id1/scan` to obtain data.

* obstacleDetect.cpp
1. By subscribing to the topic `/cloud_merge`, obstacles between the robot and the QR code can be detected.
2. This functionality only supports obstacle detection (presence or absence of obstacles) and does not provide specific information about obstacles such as their position or type.
   
### Debug Visualization Functionality
* markerVis.h 
1. During debugging, visualization information can be obtained by subscribing to the topic `/aprilnav/pose_marker`.

## Communication Protocol
### Input
| Topic Name                        | Type                        | Description               | 
| -----------                      | -----------                 | -----------               |
| /camera_id1/color/image_raw      | sensor_msgs::Image          | Camera image              |
| /rgbd_pc_id1/scan                 | sensor_msgs::LaserScan      | Data of RGB-D point cloud projected onto a 2D plane |
| /scan                            | sensor_msgs::LaserScan      | Laser scanner data        |

### Output
| Topic Name                        | Type                               | Description               | 
| -----------                      | -----------                        | -----------               |
| /aprilnav/pose_publisher         | common_msgs::autoCharge            | Positioning data for returning to the charging station |
| /aprilnav/pose_marker            | visualization_msgs::MarkerArray    | Positioning data for returning to the charging station (visualization) |

##  message type
message needs to be defined
```
float32 bias        # Horizontal deviation of the QR code
float32 distance    # Distance to the QR code
float32 angle       # Angle of the QR code
bool    stop        # Whether to stop
```
