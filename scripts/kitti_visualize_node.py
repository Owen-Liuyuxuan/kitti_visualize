#!/usr/bin/env python3
import numpy as np
import rospy 
import cv2
from utils import kitti_utils, ros_util
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool, Float32

class KittiVisualizeNode:
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        rospy.init_node("KittiVisualizeNode")
        rospy.loginfo("Starting KittiVisualizeNode.")

        self.left_image_publish  = rospy.Publisher("/kitti/left_camera/image", Image, queue_size=1, latch=True)
        self.right_image_publish = rospy.Publisher("/kitti/right_camera/image", Image, queue_size=1, latch=True)
        self.left_camera_info    = rospy.Publisher("/kitti/left_camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.right_camera_info   = rospy.Publisher("/kitti/right_camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.bbox_publish        = rospy.Publisher("/kitti/bboxes", MarkerArray, queue_size=1, latch=True)
        self.lidar_publisher     = rospy.Publisher("/kitti/lidar", PointCloud2, queue_size=1, latch=True)
        self.image_pc_publisher  = rospy.Publisher("/kitti/left_camera_pc", PointCloud2, queue_size=1, latch=True)
        
        KITTI_obj_dir = rospy.get_param("~KITTI_OBJ_DIR", None)
        KITTI_raw_dir = rospy.get_param("~KITTI_RAW_DIR", None)
        self.KITTI_depth_dir = rospy.get_param("~KITTI_DEPTH_DIR", None)
        self.image_pc_depth  = float(rospy.get_param("~Image_PointCloud_Depth", 5))
        self.update_frequency= float(rospy.get_param("~UPDATE_FREQUENCY", 8))
        if self.KITTI_depth_dir:
            self.depth_publisher = rospy.Publisher("/kitti/depth_image", PointCloud2, queue_size=1, latch=True)
        self.kitti_base_dir = [KITTI_obj_dir, KITTI_raw_dir]
        self.is_sequence = False
        self.index = 0
        self.published = False
        self.sequence_index = 0
        self.pause = False
        self.stop = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_frequency), self.publish_callback)
        rospy.Subscriber("/kitti/control/is_sequence", Bool, self.is_sequence_callback)
        rospy.Subscriber("/kitti/control/index", Int32, self.index_callback)
        rospy.Subscriber("/kitti/control/stop", Bool, self.stop_callback)
        rospy.Subscriber("/kitti/control/pause", Bool, self.pause_callback)


    def is_sequence_callback(self, msg):
        self.published=False
        self.is_sequence = msg.data
        rospy.loginfo("Switching to folder {}".format(self.kitti_base_dir[int(self.is_sequence)]))

    def stop_callback(self, msg):
        self.published=False
        self.stop = msg.data
        self.sequence_index = 0
        ros_util.clear_all_bbox(self.bbox_publish)
        self.publish_callback(None)

    def pause_callback(self, msg):
        self.pause = msg.data      
        self.published=False

    def index_callback(self, msg):
        self.index = msg.data
        self.sequence_index = 0
        self.publish_callback(None)
        
    def publish_callback(self, event):
        if self.stop: # if stopped, falls back to an empty loop
            return

        selected_folder = self.kitti_base_dir[int(self.is_sequence)]
        if selected_folder is None:
            rospy.logwarn("The selected_folder is None. Folders from launch_file:{}".format(self.kitti_base_dir))
            return
        
        meta_dict = kitti_utils.get_files(selected_folder, self.index, self.is_sequence, self.KITTI_depth_dir)
        if meta_dict is None:
            rospy.logwarn("meta_dict from kitti_utils.get_files is None, current_arguments {}"\
                .format([selected_folder, self.index, self.is_sequence, self.KITTI_depth_dir]))
            return
        P2 = meta_dict["calib"]["P2"]
        P3 = meta_dict["calib"]["P3"]
        T = meta_dict["calib"]["T_velo2cam"]
        ros_util.publish_tf(P2, P3, T)

        if not self.is_sequence:

            if meta_dict["label"]:
                objects = kitti_utils.read_labels(meta_dict["label"])
                self.bbox_publish.publish([ros_util.object_to_marker(obj, marker_id=i, duration=1.01 / self.update_frequency, color=(255, 0, 200)) for i, obj in enumerate(objects)])

            if meta_dict["additional_label"]:
                objects_add = kitti_utils.read_labels(meta_dict["additional_label"])
                self.bbox_publish.publish([ros_util.object_to_marker(obj, marker_id=i+len(objects), duration=1.01 / self.update_frequency) for i, obj in enumerate(objects_add)])

            if event is not None: # if call by timer, only the labels will get refreshed and images/point clouds are freezed
                return
            left_image = cv2.imread(meta_dict["left_image"])
            if left_image is None:
                rospy.logwarn("No detection found at {} index {}".format(self.kitti_base_dir, self.index))
                return
            ros_util.publish_image(left_image, self.left_image_publish, self.left_camera_info, P2, "left_camera")
            right_image = cv2.imread(meta_dict["right_image"])
            ros_util.publish_image(right_image, self.right_image_publish, self.right_camera_info, P3, "right_camera")

            point_cloud = np.fromfile(meta_dict["point_cloud"], dtype=np.float32).reshape(-1, 4)
            point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2]
            pitch = np.arctan2(point_cloud[:, 2], point_cloud[:, 0])
            point_cloud = point_cloud[ (point_cloud[:, 2] > -2.5) * (point_cloud[:, 2] < 1.5)]
            rgb_point_cloud = kitti_utils.color_pointcloud(point_cloud[:, :3], left_image, T, P2)
            #ros_util.publish_point_cloud(point_cloud, self.lidar_publisher, "velodyne")
            ros_util.publish_point_cloud(rgb_point_cloud, self.lidar_publisher, "velodyne", "xyzbgr")

            depth_image = np.zeros([left_image.shape[0], left_image.shape[1]])
            depth_image[:, :] = self.image_pc_depth * 256
            depth_point_cloud = ros_util.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
            ros_util.publish_point_cloud(depth_point_cloud, self.image_pc_publisher, "left_camera", 'xyzbgr')

            if "depth_image" in meta_dict and meta_dict["depth_image"] is not None:
                depth_image = cv2.imread(meta_dict["depth_image"], -1)#[H, W] uint16 /256.0=depth
                H,W,_ = left_image.shape
                print(H, W, depth_image.shape)
                depth_image = cv2.resize(depth_image, (W, H-100), interpolation=cv2.INTER_NEAREST)[50:]
                P2_depth = P2[0:3, 0:3].copy() #[3, 3]
                P2_depth[1, 2] -= 150
                depth_point_cloud = ros_util.depth_image_to_point_cloud_array(depth_image, P2_depth)
                ros_util.publish_point_cloud(depth_point_cloud, self.depth_publisher, "left_camera", 'xyz')

            
        else:
            if self.pause: # if paused, all data will freeze
                return

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

            depth_image = np.zeros([left_image.shape[0], left_image.shape[1]])
            depth_image[:, :] = self.image_pc_depth * 256
            depth_point_cloud = ros_util.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
            ros_util.publish_point_cloud(depth_point_cloud, self.image_pc_publisher, "left_camera", 'xyzbgr')

            if "depth_images" in meta_dict and meta_dict["depth_images"] is not None:
                image_name = meta_dict["left_image"][self.sequence_index].split("/")[-1]
                for depth_image_path in meta_dict["depth_images"]:
                    if image_name in depth_image_path:
                        depth_image = cv2.imread(depth_image_path, -1)#[H, W] uint16 /256.0=depth
                        depth_point_cloud = ros_util.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
                        ros_util.publish_point_cloud(depth_point_cloud, self.depth_publisher, "left_camera", 'xyzbgr')
            
            if "odom_array" in meta_dict and meta_dict["odom_array"] is not None:
                R = meta_dict["odom_array"][self.sequence_index]
                T = meta_dict["odom_array"][self.sequence_index, 0:3, 3]
                ros_util.publish_odom(R, T)

            self.sequence_index += 1

def main():
    node = KittiVisualizeNode()
    rospy.spin()

if __name__ == "__main__":
    main()
