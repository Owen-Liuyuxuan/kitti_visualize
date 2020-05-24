#!/usr/bin/env python
from fire import Fire
import numpy as np
import rospy 
import cv2
from utils import kitti_utils, ros_util
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool, Float32

class KittiVisualizeNode:
    def __init__(self, KITTI_obj_dir, KITTI_raw_dir):
        rospy.init_node("KittiVisualizeNode")
        rospy.loginfo("Starting KittiVisualizeNode.")


        self.left_image_publish  = rospy.Publisher("/kitti/left_camera/image", Image, queue_size=1, latch=True)
        self.right_image_publish = rospy.Publisher("/kitti/right_camera/image", Image, queue_size=1, latch=True)
        self.left_camera_info    = rospy.Publisher("/kitti/left_camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.right_camera_info   = rospy.Publisher("/kitti/right_camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.bbox_publish        = rospy.Publisher("/kitti/bboxes", MarkerArray, queue_size=1, latch=True)
        
        self.lidar_publisher     = rospy.Publisher("/kitti/lidar", PointCloud2, queue_size=1, latch=True)
        

        self.kitti_base_dir = [KITTI_obj_dir, KITTI_raw_dir]
        self.is_sequence = False
        self.index = 0
        self.published = False
        self.sequence_index = 0
        self.pause = False
        self.stop = True
        self.update_frequency = 4
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_frequency), self.publish_callback)
        rospy.Subscriber("/kitti/control/base_dir", String, self.base_dir_callback)
        rospy.Subscriber("/kitti/control/is_sequence", Bool, self.is_sequence_callback)
        rospy.Subscriber("/kitti/control/index", Int32, self.index_callback)
        rospy.Subscriber("/kitti/control/stop", Bool, self.stop_callback)
        rospy.Subscriber("/kitti/control/pause", Bool, self.pause_callback)

    def base_dir_callback(self, msg):
        self.kitti_base_dir = msg.data
        self.sequence_index += 1
        self.published=False
        self.publish_callback(None)

    def is_sequence_callback(self, msg):
        self.published=False
        self.is_sequence = msg.data

    def stop_callback(self, msg):
        self.published=False
        self.stop = msg.data
        self.publish_callback(None)

    def pause_callback(self, msg):
        self.pause = msg.data      
        self.published=False

    def index_callback(self, msg):
        self.index = msg.data
        self.publish_callback(None)
        
    def publish_callback(self, event):
        if self.stop:
            return
        
        meta_dict = kitti_utils.get_files(self.kitti_base_dir, self.index, self.is_sequence)
        P2 = meta_dict["calib"]["P2"]
        P3 = meta_dict["calib"]["P3"]
        T = meta_dict["calib"]["T_velo2cam"]
        ros_util.publish_tf(P2, P3, T)

        if not self.is_sequence:

            if meta_dict["label"]:
                objects = kitti_utils.read_labels(meta_dict["label"])
                self.bbox_publish.publish([ros_util.object_to_marker(obj, marker_id=i, duration=1.01 / self.update_frequency) for i, obj in enumerate(objects)])
        

            if event is not None: # call by timer
                return
            left_image = cv2.imread(meta_dict["left_image"])
            if left_image is None:
                rospy.logwarn("No detection found at {} index {}".format(self.kitti_base_dir, self.index))
                return
            ros_util.publish_image(left_image, self.left_image_publish, self.left_camera_info, P2, "left_camera")
            right_image = cv2.imread(meta_dict["right_image"])
            ros_util.publish_image(right_image, self.right_image_publish, self.right_camera_info, P3, "right_camera")

            point_cloud = np.fromfile(meta_dict["point_cloud"], dtype=np.float32).reshape(-1, 4)
            point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2 ]
            ros_util.publish_point_cloud(point_cloud, self.lidar_publisher, "velodyne")

            
        else:

            length = min([len(meta_dict["left_image"]), len(meta_dict["right_image"]), len(meta_dict["point_cloud"])])

            if length == 0:
                rospy.logwarn("No sequence found at {} index {}".format(self.kitti_base_dir, self.index))
                return
            self.sequence_index = (self.sequence_index) % length
            
            left_image = cv2.imread(meta_dict["left_image"][self.sequence_index])
            ros_util.publish_image(left_image, self.left_image_publish, self.left_camera_info, P2, "left_camera")
            right_image = cv2.imread(meta_dict["right_image"][self.sequence_index])
            ros_util.publish_image(right_image, self.right_image_publish, self.right_camera_info, P3, "right_camera")

            point_cloud = np.fromfile(meta_dict["point_cloud"][self.sequence_index], dtype=np.float32).reshape(-1, 4)
            point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2 ]
            ros_util.publish_point_cloud(point_cloud, self.lidar_publisher, "velodyne")

            if not self.pause:
                self.sequence_index += 1

def main(KITTI_obj_dir=None,
         KITTI_raw_dir=None):
    node = KittiVisualizeNode(KITTI_obj_dir, KITTI_raw_dir)
    rospy.spin()

if __name__ == "__main__":
    Fire(main)