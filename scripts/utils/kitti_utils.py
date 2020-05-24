#!/usr/bin/env python
import os
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from constants import KITTI_COLORS, KITTI_NAMES

def read_labels(file):
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

def get_files(base_dir, index, is_sequence):
    """Retrieve a dictionary of filenames, including calibration(P2, P3, R, T), left_image, right_image, point_cloud, labels(could be none)
        if is_sequence:
            Will read from KITTI raw data. 
            base_dir: str, should be absolute file path to datetime <2011_09_26>
            index: int, sequence number at that datetime <1> -> <2011_09_26_drive_0001_sync>
        else:
            Will read from KITTI detection data.
            base_dir: str, should be abolutie file path to <training/testing>
            index: int, index number for calib/image/velodyne
    """
    output_dict = {
        "calib":{
            "P2":np.zeros([3, 4]),
            "P3":np.zeros([3, 4]),
            "T_velo2cam":np.zeros([3, 4]),
        },
        "left_image":"",
        "right_image":"",
        "point_cloud":"",
        "label":None
    }
    if is_sequence:
        cam_calib_file = os.path.join(base_dir, "calib_cam_to_cam.txt")
        P2, P3 = read_P23_from_sequence(cam_calib_file)
        velo_calib_file = os.path.join(base_dir, "calib_velo_to_cam.txt")
        T_velo2cam = read_T_from_sequence(velo_calib_file)
        output_dict["calib"]["P2"] = P2
        output_dict["calib"]["P3"] = P3
        output_dict["calib"]["T_velo2cam"] = T_velo2cam

        sequences = os.listdir(base_dir)
        sequences.sort()
        sequences = [sequence for sequence in sequences if not sequence.endswith("txt")]
            
        sequence = sequences[index%len(sequences)]
        left_dir = os.path.join(base_dir, sequence, "image_02", "data")
        left_images = os.listdir(left_dir)
        left_images.sort()
        left_images = [os.path.join(left_dir, left_image) for left_image in left_images]

        right_dir = os.path.join(base_dir, sequence, "image_03", "data")
        right_images= os.listdir(right_dir)
        right_images.sort()
        right_images = [os.path.join(right_dir, right_image) for right_image in right_images]

        velo_dir = os.path.join(base_dir, sequence, "velodyne_points", "data")
        velodynes = os.listdir(velo_dir)
        velodynes.sort()
        velodynes = [os.path.join(velo_dir, velodyne) for velodyne in velodynes]
        output_dict["left_image"] = left_images
        output_dict["right_image"] = right_images
        output_dict["point_cloud"] = velodynes

    else:
        kitti_ind = "%06d" % index
        left_image = os.path.join(base_dir, "image_2", kitti_ind + ".png")
        right_image = os.path.join(base_dir, "image_3", kitti_ind + ".png")
        point_cloud = os.path.join(base_dir, "velodyne", kitti_ind + ".bin")
        calib_file = os.path.join(base_dir, "calib", kitti_ind + ".txt")
        label_file = os.path.join(base_dir, "label_2", kitti_ind + ".txt")
        P2, P3, T_velo2cam = read_calib_from_detection(calib_file)

        output_dict["left_image"] = left_image
        output_dict["right_image"] = right_image
        output_dict["point_cloud"] = point_cloud
        output_dict["calib"]["P2"] = P2
        output_dict["calib"]["P3"] = P3
        output_dict["calib"]["T_velo2cam"] = T_velo2cam

        if os.path.isfile(label_file):
            output_dict["label"] = label_file

    return output_dict

        
if __name__ == "__main__":
    output_dict1 = get_files("/home/hins/kitti/2011_09_26", 1, True)
    output_dict2 = get_files("/home/hins/Desktop/M3D-RPN/data/kitti/training", 1, False)
    a = 1
