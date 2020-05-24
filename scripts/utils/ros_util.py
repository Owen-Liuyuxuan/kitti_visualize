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

def depth_image_to_point_cloud_array(depth_image, K, parent_frame="left_camera", rgb_image=None):
    """  convert depth image into color pointclouds [xyzbgr]
    
    """
    depth_image = np.copy(depth_image) / 256.0
    w_range = np.arange(0, depth_image.shape[1], dtype=np.float32)
    h_range = np.arange(0, depth_image.shape[0], dtype=np.float32)
    w_grid, h_grid = np.meshgrid(w_range, h_range) #[H, W]
    K_expand = np.eye(4)
    K_expand[0:3, 0:3] = K
    K_inv = np.linalg.inv(K_expand) #[4, 4]

    #[H, W, 4, 1]
    expand_image = np.stack([w_grid * depth_image, h_grid * depth_image, depth_image, np.ones_like(depth_image)], axis=2)[...,np.newaxis]

    pc_3d = np.matmul(K_inv, expand_image)[..., 0:3, 0] #[H, W, 3]
    if rgb_image is not None:
        pc_3d = np.concatenate([pc_3d, rgb_image/256.0], axis=2).astype(np.float32)
    point_cloud = pc_3d[depth_image > 0,:]
    
    return point_cloud


def object_to_marker(obj, frame_id="base", marker_id=None, duration=0.15):
    """ Transform an object to a marker.

    Args:
        obj: Dict
        frame_id: string; parent frame name
        marker_id: visualization_msgs.msg.Marker.id
        duration: the existence time in rviz
    
    Return:
        marker: visualization_msgs.msg.Marker

    object dictionary expectation:
        object['whl'] = [w, h, l]
        object['xyz'] = [x, y, z] # center point location in center camera coordinate
        object['theta']: float
        object['score']: float
        object['type_name']: string # should have name in constant.KITTI_NAMES

    """
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

def publish_tf(P2=None, P3=None, T=None):
    """ Publish camera, velodyne transform from P2, P3, T
    """
    br = tf.TransformBroadcaster()

    ## broadcast translation and rotation in velodyne T
    if T is not None:
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
        # tf.transformations.quaternion_from_matrix(homo_R.T),
        [0.5, -0.5, 0.5, -0.5],
        rospy.Time.now(),
        "base",
        "world"
    )
    if P2 is not None:
    ## broadcast the translation in P2
        br.sendTransform(
            (-P2[0, 3] / P2[0, 0], 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "left_camera",
            "base"
        )
    if P3 is not None:
        ## broadcast translation in P3
        br.sendTransform(
            (-P3[0, 3] / P3[0, 0], 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "right_camera",
            "base"
        )

def publish_image(image, image_publisher, camera_info_publisher, P, frame_id):
    """Publish image and info message to ROS.

    Args:
        image: numpy.ndArray.
        image_publisher: rospy.Publisher
        camera_info_publisher: rospy.Publisher, should publish CameraInfo
        P: projection matrix [3, 4]. though only [3, 3] is useful.
        frame_id: string, parent frame name.
    """
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

def array2pc2(points, parent_frame, field_names='xyza'):
    """ Creates a point cloud message.
    Args:
        points: Nxk array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
        field_names : name for the k channels repectively i.e. "xyz" / "xyza"
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(field_names)]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * len(field_names)),
        row_step=(itemsize * len(field_names) * points.shape[0]),
        data=data
    )

def publish_point_cloud(pointcloud, pc_publisher, frame_id, field_names='xyza'):
    """Convert point cloud array to PointCloud2 message and publish
    
    Args:
        pointcloud: point cloud array [N,3]/[N,4]
        pc_publisher: ROS publisher for PointCloud2
        frame_id: parent_frame name.
        field_names: name for each channel, ['xyz', 'xyza'...]
    """
    msg = array2pc2(pointcloud, frame_id, field_names)
    pc_publisher.publish(msg)
