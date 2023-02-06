#!/usr/bin/env python
import os
import numpy as np
import rospy
from visualization_msgs.msg import Marker
import scipy.io as sio
from .constants import KITTI_COLORS, KITTI_NAMES

def color_pointcloud(pts, image, T, P2):
    hfiller = np.expand_dims(np.ones(pts.shape[0]), axis=1)
    pts_hT = np.hstack((pts, hfiller)).T #(4, #pts)
    if T.shape == (3, 4):
        T1 = np.eye(4)
        T1[0: 3] = T.copy()
    else:
        T1 = T.copy()
    pts_cam_T = T1.dot(pts_hT) # (4, #pts)

    pixels_T = P2.dot(pts_cam_T) #(3, #pts)
    pixels = pixels_T.T
    pixels[:, 0] /= pixels[:, 2] + 1e-6
    pixels[:, 1] /= pixels[:, 2] + 1e-6
    w_coordinate = pixels[:, 0].astype(np.int32)
    h_coordinate = pixels[:, 1].astype(np.int32)
    w_coordinate[w_coordinate <= 0] = 0
    w_coordinate[w_coordinate >= image.shape[1]] = image.shape[1] - 1
    h_coordinate[h_coordinate <= 0] = 0
    h_coordinate[h_coordinate >= image.shape[0]] = image.shape[0] - 1
    
    bgr = image[h_coordinate, w_coordinate, :]/ 256.0
    return np.concatenate([pts, bgr], axis=1).astype(np.float32)

def read_labels(file):
    """Read objects 3D bounding boxes from a label file.

    Args:
        file: string of path to the label file

    Returns:
        objects: List[Dict];
        object['whl'] = [w, h, l]
        object['xyz'] = [x, y, z] # center point location in center camera coordinate
        object['theta']: float
        object['score']: float
        object['type_name']: string
    """
    objects = []
    with open(file, 'r') as f:
        for line in f.readlines():
            objdata = line.split()
            class_name = objdata[0]
            if class_name in KITTI_NAMES:
                whl = [float(objdata[9]), float(objdata[8]), float(objdata[10])]
                xyz = [float(objdata[11]), float(objdata[12]) - 0.5 * whl[1], float(objdata[13])]
                theta = float(objdata[14])
                if len(objdata) > 15:
                    score = float(objdata[15])
                else:
                    score = 1.0
                objects.append(
                    dict(whl=whl, xyz=xyz, theta=theta, type_name=class_name, score=score)
                )
    return objects

def read_calib_from_detection(file):
    """ read P2, P3, T from a detection calibration file
    """
    P2 = None
    P2 = None
    R0_rect = None
    Tr_velo2cam = None
    with open(file, 'r') as f:
        for line in f.readlines():
            if line.startswith("R0_rect:"):
                data = line.split(" ")
                R0_rect = np.array([float(x) for x in data[1:10]])
                R0_rect = np.reshape(R0_rect, [3, 3])
            if line.startswith("Tr_velo_to_cam:"):
                data = line.split(" ")
                Tr_velo2cam = np.array([float(x) for x in data[1:13]])
                Tr_velo2cam = np.reshape(Tr_velo2cam, [3, 4]) 
            if line.startswith("P2:"):
                data = line.split(" ")
                P2 = np.array([float(x) for x in data[1:13]])
                P2 = np.reshape(P2, [3, 4])
            if line.startswith("P3:"):
                data = line.split(" ")
                P3 = np.array([float(x) for x in data[1:13]])
                P3 = np.reshape(P3, [3, 4]) 
    assert Tr_velo2cam is not None, "can not find T_velo2cam in file {}".format(file)
    assert R0_rect is not None, "can not find R in file {}".format(file)
    assert P2 is not None, "can not find P2 in file {}".format(file)
    assert P3 is not None, "can not find P3 in file {}".format(file)

    T_velo2cam = np.dot(R0_rect, Tr_velo2cam)
    return P2, P3, T_velo2cam

def read_T_from_sequence(file):
    """ read T from a sequence file calib_velo_to_cam.txt
    """
    R = None
    T = None
    with open(file, 'r') as f:
        for line in f.readlines():
            if line.startswith("R:"):
                data = line.split(" ")
                R = np.array([float(x) for x in data[1:10]])
                R = np.reshape(R, [3, 3])
            if line.startswith("T:"):
                data = line.split(" ")
                T = np.array([float(x) for x in data[1:4]])
                T = np.reshape(T, [3, 1]) 
    assert R is not None, "can not find R in file {}".format(file)
    assert T is not None, "can not find T in file {}".format(file)

    T_velo2cam = np.eye(4)
    T_velo2cam[0:3, 0:3] = R
    T_velo2cam[0:3, 3:4] = T
    return T_velo2cam

def read_Timu_from_sequence(file):
    R = None
    T = None
    with open(file, 'r') as f:
        for line in f.readlines():
            if line.startswith("R:"):
                data = line.split(" ")
                R = np.array([float(x) for x in data[1:10]])
                R = np.reshape(R, [3, 3])
            if line.startswith("T:"):
                data = line.split(" ")
                T = np.array([float(x) for x in data[1:4]])
                T = np.reshape(T, [3, 1]) 
    assert R is not None, "can not find R in file {}".format(file)
    assert T is not None, "can not find T in file {}".format(file)
    T_imu2velo = np.eye(4)
    T_imu2velo[0:3, 0:3] = R
    T_imu2velo[0:3, 3:4] = T
    return T_imu2velo

def read_P23_from_sequence(file):
    """ read P2 and P3 from a sequence file calib_cam_to_cam.txt
    """
    P2 = None
    P3 = None
    with open(file, 'r') as f:
        for line in f.readlines():
            if line.startswith("P_rect_02"):
                data = line.split(" ")
                P2 = np.array([float(x) for x in data[1:13]])
                P2 = np.reshape(P2, [3, 4])
            if line.startswith("P_rect_03"):
                data = line.split(" ")
                P3 = np.array([float(x) for x in data[1:13]])
                P3 = np.reshape(P3, [3, 4])
    assert P2 is not None, "can not find P2 in file {}".format(file)
    assert P3 is not None, "can not find P3 in file {}".format(file)
    return P2, P3

def determine_date_index(base_dir, index):
    date_times = os.listdir(base_dir)
    residual_index = index
    date_times.sort()
    for date_time in date_times:
        date_dir = os.path.join(base_dir, date_time)
        sequences_at_date = [file for file in os.listdir(date_dir) if not file.endswith("txt")]
        sequences_at_date.sort()
        if residual_index >= len(sequences_at_date):
            residual_index = residual_index - len(sequences_at_date)
        else:
            break
    else:
        rospy.logwarn("index {} is larger than the total number of sequences {} in {}".format(index, index-residual_index, base_dir))
        return None, None, None
    
    return date_dir, sequences_at_date[residual_index]

def get_files(base_dir, index, is_sequence, depth_dir=None):
    """Retrieve a dictionary of filenames, including calibration(P2, P3, R, T), left_image, right_image, point_cloud, labels(could be none)

        if is_sequence:
            Will read from KITTI raw data. 
            base_dir: str, should be absolute file path to kitti_raw
            index: int, sequence number at that datetime <1> -> <2011_09_26_drive_0001_sync>
        else:
            Will read from KITTI detection data.
            base_dir: str, should be abolutie file path to <training/testing>
            index: int, index number for calib/image/velodyne
    """
    output_dict = {
        "calib":{
            "P2":None,
            "P3":None,
            "T_velo2cam":None,
            "T_imu2velo":None,
        },
        "left_image":"",
        "right_image":"",
        "point_cloud":"",
        "label":None,
        "additional_label":None,
        "odom_array":None,
    }
    if is_sequence:
        
        date_dir, sequence = determine_date_index(base_dir, index)
        cam_calib_file = os.path.join(date_dir, "calib_cam_to_cam.txt")
        P2, P3 = read_P23_from_sequence(cam_calib_file)
        velo_calib_file = os.path.join(date_dir, "calib_velo_to_cam.txt")
        T_velo2cam = read_T_from_sequence(velo_calib_file)
        imu_calib_file = os.path.join(date_dir, 'calib_imu_to_velo.txt')
        T_imu2velo = read_Timu_from_sequence(imu_calib_file)
        output_dict["calib"]["P2"] = P2
        output_dict["calib"]["P3"] = P3
        output_dict["calib"]["T_velo2cam"] = T_velo2cam
        output_dict["calib"]["T_imu2velo"] = T_imu2velo

        left_dir = os.path.join(date_dir, sequence, "image_02", "data")
        left_images = os.listdir(left_dir)
        left_images.sort()
        left_images = [os.path.join(left_dir, left_image) for left_image in left_images]

        right_dir = os.path.join(date_dir, sequence, "image_03", "data")
        right_images= os.listdir(right_dir)
        right_images.sort()
        right_images = [os.path.join(right_dir, right_image) for right_image in right_images]

        velo_dir = os.path.join(date_dir, sequence, "velodyne_points", "data")
        velodynes = os.listdir(velo_dir)
        velodynes.sort()
        velodynes = [os.path.join(velo_dir, velodyne) for velodyne in velodynes]


        odometry_mat = os.path.join(date_dir, sequence, 'oxts', 'pose.mat') # pose mat can be generated with official matlab toolkits
        if os.path.isfile(odometry_mat):
            pose_dict = sio.loadmat(odometry_mat)
            odom_array = pose_dict[[key for key in list(pose_dict.keys()) if not key.startswith('__')][0]] # the only non "__" key, should be a N, 4, 4 matrix
        else:
            odom_array = np.stack([np.eye(4) for _ in range(len(left_images))], dim=0) # just not outputing odom is also fine.

        if not depth_dir is None:
            if sequence in os.listdir(os.path.join(depth_dir, "train")):
                depth_image_dir = os.path.join(depth_dir, "train", sequence, 'proj_depth', 'groundtruth', 'image_02')
                depth_images = os.listdir(depth_image_dir)
                depth_images.sort()
                depth_images = [os.path.join(depth_image_dir, depth_image) for depth_image in depth_images]
                output_dict["depth_images"] = depth_images
            if sequence in os.listdir(os.path.join(depth_dir, "val")):
                depth_image_dir = os.path.join(depth_dir, "val", sequence, 'proj_depth', 'groundtruth', 'image_02')
                depth_images = os.listdir(depth_image_dir)
                depth_images.sort()
                depth_images = [os.path.join(depth_image_dir, depth_image) for depth_image in depth_images]
                output_dict["depth_images"] = depth_images
                rospy.loginfo("the current sequence {} is in validation set".format(sequence))

        output_dict["left_image"] = left_images
        output_dict["right_image"] = right_images
        output_dict["point_cloud"] = velodynes
        output_dict["odom_array"] = odom_array

    else:
        kitti_ind = "%06d" % index
        left_image = os.path.join(base_dir, "image_2", kitti_ind + ".png")
        right_image = os.path.join(base_dir, "image_3", kitti_ind + ".png")
        point_cloud = os.path.join(base_dir, "velodyne", kitti_ind + ".bin")
        calib_file = os.path.join(base_dir, "calib", kitti_ind + ".txt")
        label_file = os.path.join(base_dir, "label_2", kitti_ind + ".txt")
        additional_label_file = os.path.join(base_dir, "additional_label_2", kitti_ind + ".txt")
        depth_file = os.path.join(base_dir, "depth", kitti_ind + ".png")
        P2, P3, T_velo2cam = read_calib_from_detection(calib_file)

        output_dict["left_image"] = left_image
        output_dict["right_image"] = right_image
        output_dict["point_cloud"] = point_cloud
        output_dict["calib"]["P2"] = P2
        output_dict["calib"]["P3"] = P3
        output_dict["calib"]["T_velo2cam"] = T_velo2cam

        if os.path.isfile(label_file):
            output_dict["label"] = label_file

        if os.path.isfile(depth_file):
            output_dict["depth_image"] = depth_file
        
        if os.path.isfile(additional_label_file):
            output_dict["additional_label"] = additional_label_file

    return output_dict

        
if __name__ == "__main__":
    output_dict1 = get_files("/home/hins/kitti/2011_09_26", 1, True)
    output_dict2 = get_files("/home/hins/Desktop/M3D-RPN/data/kitti/training", 1, False)
    a = 1
