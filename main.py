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

from src.platform.platform import Platform, deep_get


class Ui(QMainWindow):
    def __init__(self):
        super().__init__()

        uic.loadUi(f'{os.getcwd()}/ihm_tests/ui/main.ui', self)

        self.platform = None

        self.config_button.clicked.connect(self.load_conf_file)
        self.start_stop_connexion.clicked.connect(self.connect_robot)
        self.venturi.clicked.connect(self.venturi_state)
        self.init.clicked.connect(self.go_to_init)

        self.showMaximized()
        self.show()

    def load_conf_file(self):
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        if dlg.exec_():
            filename = dlg.selectedFiles()
            self.platform = Platform(filename[0])

    def connect_robot(self):
        success, message = self.platform.robot.connexion()
        return success, message

    def go_to_init(self):
        success, message = self.platform.robot.go_to_init()
        # point = next((x for x in self.platform.points if x['name'] == "initial_position"), None)
        # coord = deep_get(point, ["data", "coord"], None)
        # success, message = self.platform.robot.articular_trajectory(coord)
        # print(success, message)

    def go_to_camera(self):
        print('i am going to the camera ')

    def venturi_state(self):
        print("i am changing the venturi state")
        venturi = next((obj for obj in self.platform.devices if obj.name == 'venturi'), None)
        venturi.inpulse_pin_state(5)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
