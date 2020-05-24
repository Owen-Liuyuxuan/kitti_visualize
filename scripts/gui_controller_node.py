#!/usr/bin/env python
import rospy 
import sys
from PyQt5 import QtWidgets
from std_msgs.msg import Bool, String, Int32, Float32
from qt_utils.gui import Ui_Dialog

class RosInterface(object):
    def __init__(self):
        super(RosInterface, self).__init__()
        self.index_pub = rospy.Publisher("/kitti/control/index", Int32, queue_size=1)
        self.base_dir_pub = rospy.Publisher("/kitti/control/base_dir", String, queue_size=1)
        self.in_sequence_pub = rospy.Publisher("/kitti/control/is_sequence", Bool, queue_size=1)
        self.pause_pub = rospy.Publisher("/kitti/control/pause", Bool, queue_size=1)
        self.stop_pub = rospy.Publisher("/kitti/control/stop", Bool, queue_size=1)

    def publish_index(self, index):
        self.index_pub.publish(Int32(index))

    def publish_directory(self, directory):
        self.base_dir_pub.publish(String(directory))

    def publish_is_squence_bool(self, is_sequence):
        self.in_sequence_pub.publish(Bool(is_sequence))

    def publish_stop_bool(self, boolean):
        self.stop_pub.publish(Bool(boolean))

    def publish_pause_bool(self, boolean):
        self.pause_pub.publish(Bool(boolean))


class GUIControllerNode:
    def __init__(self):
        rospy.init_node("GUI_controller_node")
        rospy.loginfo("Starting GUI_controller_node.")
        self.qt_processor = Ui_Dialog(RosInterface())

    def start(self):
        while not rospy.is_shutdown() and not self.qt_processor.quit:
            app = QtWidgets.QApplication(sys.argv)
            diag = QtWidgets.QDialog()
            self.qt_processor.setupUi(diag)
            diag.show()
            app.exec_()
            app = None



if __name__ == "__main__":
    ros_node = GUIControllerNode()
    ros_node.start()