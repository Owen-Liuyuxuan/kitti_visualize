from PyQt5 import QtCore, QtGui, QtWidgets
import os

class Ui_Dialog(object):
    def __init__(self, ros_interface):
        self.ros_interface = ros_interface
        self.reset()

    def accept(self):
        self.base_dir_edited()

    def reject(self):
        self.quit = True
        self.Dialog.reject()

    def reset(self):
        self.quit = False
        self.text_set = False

    def base_dir_edited(self):
        line = self.lineEdit.text()
        if os.path.isdir(line):
            self.ros_interface.publish_directory(line)
            self.text_set = True
            if not self.checkBox.isChecked() and not self.checkBox_2.isChecked():
                max_file = len(os.listdir(os.path.join(line, "image_2")))
                self.progressBar.setMaximum(max_file)
            else:
                childs = os.listdir(line)
                childs = [child for child in childs if not child.endswith("txt")]
                self.progressBar.setMaximum(len(childs))


    def img_index_changed(self):
        value = self.imageIndex.value()
        if self.text_set:
            self.progressBar.setProperty("value", value)
        self.ros_interface.publish_index(value)

    def is_sequence_changed(self):
        bool_value = self.checkBox.isChecked()
        self.ros_interface.publish_is_squence_bool(bool_value)

    def stop_changed(self):
        bool_value = self.checkBox_2.isChecked()
        self.ros_interface.publish_stop_bool(bool_value)

    def pause_changed(self):
        bool_value = self.checkBox_3.isChecked()
        self.ros_interface.publish_pause_bool(bool_value)


    def setupUi(self, Dialog):
        self.Dialog = Dialog
        Dialog.setObjectName("Dialog")
        Dialog.resize(400, 225)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(310, 160, 81, 241))
        self.buttonBox.setOrientation(QtCore.Qt.Vertical)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.imageIndex = QtWidgets.QSpinBox(Dialog)
        self.imageIndex.setGeometry(QtCore.QRect(80, 40, 81, 41))
        self.imageIndex.setMaximum(1000000)
        self.imageIndex.setObjectName("imageIndex")
        self.imageIndex.valueChanged.connect(self.img_index_changed)
        self.lineEdit = QtWidgets.QLineEdit(Dialog)
        self.lineEdit.setGeometry(QtCore.QRect(60, 110, 311, 21))
        self.lineEdit.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setText("/home/hins/Desktop/M3D-RPN/data/kitti/training")
        # self.lineEdit.editingFinished.connect(self.base_dir_edited)

        self.progressBar = QtWidgets.QProgressBar(Dialog)
        self.progressBar.setGeometry(QtCore.QRect(30, 10, 361, 21))
        self.progressBar.setProperty("value", 0)
        self.progressBar.setFormat("%v/%m")
        self.progressBar.setObjectName("progressBar")


        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(40, 50, 51, 21))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(0, 110, 67, 17))
        self.label_2.setObjectName("label_2")
        self.checkBox = QtWidgets.QCheckBox(Dialog)
        self.checkBox.setGeometry(QtCore.QRect(200, 40, 161, 41))
        self.checkBox.setObjectName("checkBox")
        self.checkBox.toggled.connect(self.is_sequence_changed)

        self.checkBox_2 = QtWidgets.QCheckBox(Dialog)
        self.checkBox_2.setGeometry(QtCore.QRect(30, 150, 131, 41))
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_2.setChecked(True)
        self.checkBox_2.toggled.connect(self.stop_changed)

        self.checkBox_3 = QtWidgets.QCheckBox(Dialog)
        self.checkBox_3.setGeometry(QtCore.QRect(110, 150, 161, 41))
        self.checkBox_3.setChecked(False)
        self.checkBox_3.setObjectName("checkBox_3")
        self.checkBox_3.toggled.connect(self.pause_changed)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.progressBar.setFormat(_translate("Dialog", "%v/%m"))
        self.label.setText(_translate("Dialog", "index"))
        self.label_2.setText(_translate("Dialog", "base_dir"))
        self.checkBox.setText(_translate("Dialog", "IsSequential"))
        self.checkBox_2.setText(_translate("Dialog", "Stop"))
        self.checkBox_3.setText(_translate("Dialog", "Pause(for sequence)"))
