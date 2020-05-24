# Kitti Visualization

Ros package to visualize KITTI object data, raw data, and depth prediction data with RVIZ

## Getting Started:

Overwrite the folder names in the launch file.

## GUI

![image](docs/gui.png)

### User manual:

    index: integer selection notice do not overflow the index number (especially for kitti object dataset)

    isSequential: boolean flag to determine if we are using streaming data (for raw dataset)

    Stop: stop any data loading or processing of the visualization node.
    
    Pause: prevent pointer of the sequantial data stream from increasing, keep the current scene.

    Cancel: quit.

## Object Detection

Following the file structure of kitti object detection dataset. Point clouds, stereo images, object labels are loaded and published

![image](docs/object.png)

## Raw Data & Depth Prediction Dataset

We support video-like streaming raw data. Depth Prediction dataset follows similar structure of raw data, thus can be visualized in RGB point clouds together(optionally). 

![image](docs/sequence.png)

