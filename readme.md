# Kitti Visualization

Ros package to visualize KITTI object data, raw data, and depth prediction data with RVIZ

## Getting Started:

### Data Preparation

You can use this repo with either or both KITTI [object dataset](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) and [raw dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php).  The code will not raise error until the data is actually being read, so if you are using only object dataset or only raw dataset, the code will still work fine (we have a GUI to control which dataset we are using). 

### Software Prerequisite

This repo runs with ROS python3 (noetic), and we expect PyQt5 correctly setup with ROS installation.

If you would like to run with ROS python2 (melodic). Please checkout to older code (we only modify import standard for different python):
```bash
git checkout a42df15e574119221b833547cb3868ba590012f8
```

Clone the repo under the {workspace}/src/ folder. Overwrite the folder names in the [launch file](./launch/visualize_launch.launch) to point to your data. 

```bash
cd catkin_ws/src
git clone https://github.com/Owen-Liuyuxuan/kitti_visualize
cd ..
source devel/setup.bash # devel/setup.zsh or devel/setup.sh for your own need.

# modify and check the data path!! Also control the publishing frequency of the data stream.
nano src/kitti_visualize/launch/visualize_launch.launch 

# this will launch the data publisher / rviz / GUI controller
roslaunch kitti_visualize visualize_launch.launch 
```

### Core Features:

- [x] KITTI object detection dataset support. 
- [x] KITTI raw data sequence support. 
- [x] KITTI depth prediction support. 
- [x] Stereo RGB cameras.
- [x] Filtered LiDAR, RGB point clouds.
- [x] Ground truth bounding boxes.
- [x] TF-tree (camera and LiDAR).
- [x] GUI control & ROS topic control.
- [ ] IMU tf trees.

## GUI

![image](docs/gui.png)

### User manual:

    index: integer selection notice do not overflow the index number (especially for kitti object dataset)

    isSequential: boolean flag to determine if we are using streaming data (for raw dataset)

    Stop: stop any data loading or processing of the visualization node.
    
    Pause: prevent pointer of the sequantial data stream from increasing, keep the current scene.

    Cancel: quit. (click this before killing the entire launch process)

## Object Detection

Following the file structure of kitti object detection dataset. Point clouds, stereo images, object labels are loaded and published

![image](docs/object.png)

## Raw Data & Depth Prediction Dataset

We support video-like streaming raw data. Depth Prediction dataset follows similar structure of raw data, thus can be visualized in RGB point clouds together(optionally). 

![image](docs/sequence.png)

## ROS Topics

### Published Topics

/kitti/left_camera/image ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

/kitti/right_camera/image ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

/kitti/left_camera/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

/kitti/right_camera/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))

/kitti/lidar ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))

/kitti/left_camera_pc ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))

/kitti/bboxes ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

The tf trees are also well constructed. We have a predefined rviz file for visualizing all topics and tf trees.

### Beta Features:

Features that are supported with some tricks.

- [x] Additional labels allowed to visualize detection results along with GT bboxes.
- [x] Odometry supported in sequence reading and will publish to the TF tree.

#### Additional labels
To visualize results predicted by some existing object detection algorithms/repo like [visualDet3D](https://github.com/Owen-Liuyuxuan/visualDet3D), we first have their predictions on training split/test split stored in text files, as required by the KITTI submissions. Then we copy the predictions to kitti_obj/{split}/additional_label_2. Then the prediction results will be published with gt labels on the same topic.


