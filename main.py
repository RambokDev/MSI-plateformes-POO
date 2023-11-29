#!/usr/bin/python3
import atexit
import subprocess
import cv2
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QMessageBox, QFileDialog
import sys
import os
from numpy import size

from platform.Platform import Platform


class Ui(QtWidgets.QMainWindow, Platform):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi(f'{os.getcwd()}/ihm_tests/ui/main.ui', self)
        self.config_button.clicked.connect(self.getfiles)
        self.platform = Platform()
        self.showMaximized()
        self.show()

    def getfiles(self):
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)
        if dlg.exec_():
            filename = dlg.selectedFiles()
            self.platform.initialisation(filename[0])


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
