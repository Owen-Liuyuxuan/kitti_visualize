#!/usr/bin/env python
import rospy 
import numpy as np
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool
import tf
from cv_bridge import CvBridge
import cv2
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from .constants import KITTI_NAMES, KITTI_COLORS


def object_to_marker(obj, frame_id="base", marker_id=None, duration=0.15):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame_id
    if marker_id is not None:
        marker.id = marker_id
    marker.type = 1
    marker.pose.position.x = obj["xyz"][0]
    marker.pose.position.y = obj["xyz"][1]
    marker.pose.position.z = obj["xyz"][2]
    
    q = tf.transformations.quaternion_from_euler(0, obj["theta"], 0)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.scale.x = obj["whl"][2]
    marker.scale.y = obj["whl"][1]
    marker.scale.z = obj["whl"][0]

    object_cls_index = KITTI_NAMES.index(obj["type_name"])
    obj_color = KITTI_COLORS[object_cls_index] #[r, g, b]
    marker.color.r = obj_color[0] / 255.0
    marker.color.g = obj_color[1] / 255.0
    marker.color.b = obj_color[2] / 255.0
    marker.color.a = obj["score"] * 0.5

    marker.lifetime = rospy.Duration.from_sec(duration)
    return marker

def publish_tf(P2, P3, T):
    """ Publish camera, velodyne transform from P2, P3, T
    """
    br = tf.TransformBroadcaster()

    ## broadcast translation and rotation in velodyne T
    homo_R = np.eye(4)
    homo_R[0:3, 0:3] = T[0:3, 0:3]
    br.sendTransform(
        (T[0, 3], T[1, 3], T[2, 3]),
        tf.transformations.quaternion_from_matrix(homo_R),
        rospy.Time.now(),
        "velodyne",
        "base"
    )

    ## broadcast the translation from world to base

    br.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_matrix(homo_R.T),
        rospy.Time.now(),
        "base",
        "world"
    )

    ## broadcast the translation in P2
    br.sendTransform(
        (-P2[0, 3] / P2[0, 0], 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "left_camera",
        "base"
    )

    ## broadcast translation in P3
    br.sendTransform(
        (-P3[0, 3] / P3[0, 0], 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "right_camera",
        "base"
    )

def publish_image(image, image_publisher, camera_info_publisher, P, frame_id):
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    image_msg.header.frame_id = frame_id
    image_msg.header.stamp = rospy.Time.now()
    image_publisher.publish(image_msg)

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = rospy.Time.now()
    camera_info_msg.height = image.shape[0]
    camera_info_msg.width = image.shape[1]
    camera_info_msg.D = [0, 0, 0, 0, 0]
    camera_info_msg.K = np.reshape(P[0:3, 0:3], (-1)).tolist()
    P_no_translation = np.zeros([3, 4])
    P_no_translation[0:3, 0:3] = P[0:3, 0:3]
    camera_info_msg.P = np.reshape(P_no_translation, (-1)).tolist()

    camera_info_publisher.publish(camera_info_msg)

def array2pc2(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyza')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 4),
        row_step=(itemsize * 4 * points.shape[0]),
        data=data
    )

def publish_point_cloud(pointcloud, pc_publisher, frame_id):
    msg = array2pc2(pointcloud, frame_id)
    pc_publisher.publish(msg)