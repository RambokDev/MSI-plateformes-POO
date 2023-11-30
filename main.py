#!/usr/bin/python3
import atexit
import subprocess
import cv2
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QFileDialog, QMainWindow
import sys
import os
from numpy import size

from src.ihm_tests.camera.camera_connexion import camera_basler
from src.platform.platform import Platform


class Ui(QMainWindow):
    def __init__(self):
        super().__init__()

        uic.loadUi(f'{os.getcwd()}/src/ihm_tests/ui/main.ui', self)
        self.platform = None
        self.config_button.clicked.connect(self.load_conf_file)
        self.start_stop_connexion.clicked.connect(self.connect_robot)
        self.venturi.clicked.connect(lambda x: self.toogle_pin('venturi'))
        self.bin_lights.clicked.connect(lambda x: self.impulse_pin('trigger_cam_bac'))
        self.camera_lights.clicked.connect(lambda x: self.impulse_pin('trigger_cam_angle'))
        self.init.clicked.connect(lambda x: self.go_to_pose("initial_position"))
        self.camera.clicked.connect(lambda x: self.go_to_pose("camera_angle"))
        self.bin.clicked.connect(lambda x: self.go_to_pose("bin"))
        self.take_image.clicked.connect(self.show_image)
        self.take_image_angle.clicked.connect(self.show_image_angle)
        self.imageDeBase = None
        self.filename_config = None
        self.filename = None
        self.showMaximized()
        self.show()

    def load_conf_file(self):
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        if dlg.exec_():
            filename = dlg.selectedFiles()
            self.platform = Platform(filename[0])
            self.filename_config = filename[0]

    def connect_robot(self):
        success, message = self.platform.robot.connexion()
        return success, message

    def go_to_pose(self, name):
        if self.filename:
            trajectory = next((x for x in self.platform.trajectories if x.name == name), None)
            success, message = self.platform.robot.go_to(trajectory)
            print(success, message)
        else:
            raise ValueError("Please load a config file ")

    def toogle_pin(self, device_name):
        toggle = next((obj for obj in self.platform.devices if obj.name == device_name), None)
        toggle.toggle_pin_state()

    def impulse_pin(self, device_name):
        impulse = next((obj for obj in self.platform.devices if obj.name == device_name), None)
        impulse.inpulse_pin_state(0.5)

    # for the Images
    def show_image_angle(self):
        """
        This function is called in order to display the image in angle tab
        """
        print("=====Display image Angle =====")
        img_name_angle = camera_basler(0)
        self.image_angle.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(0)
        image = cv2.imread(img_name_angle)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image_angle.setPixmap(QPixmap.fromImage(qImg))

    def show_image(self):
        """
        This function is called in order to display the image in pickup tab
        """
        print("=====Display image =====")
        img_name = camera_basler(1)
        self.filename = img_name
        self.image.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(1)
        image = cv2.imread(img_name)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image.setPixmap(QPixmap.fromImage(qImg))
        self.image.mousePressEvent = self.getPos

    def compute_ratio(self, camera_type):
        """
        This function compute for you the ratio in order to display the right image in the qt label.
        :param camera_type: the camera type 0 or 1
        """
        if camera_type == 0:
            W = self.image_angle.width()
            H = self.image_angle.height()
        else:
            W = self.image.width()
            H = self.image.height()

        self.imageDeBase = cv2.imread(self.filename)

        width = size(self.imageDeBase, 1)
        height = size(self.imageDeBase, 0)
        ratio = min(W / width, H / height)
        print(ratio, W, H, width, height)
        return ratio, width, height


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
