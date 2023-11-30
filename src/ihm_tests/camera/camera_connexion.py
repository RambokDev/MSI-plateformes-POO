#!/usr/bin/python3
import time
from numpy import size
import cv2
from pypylon import pylon

ON, OFF = 1, 0


def camera_basler(type_camera):
    """
    This function is called for the camera Basler connexion
    :param  type_camera : 0 angle Camera , 1 pickup Camera
    """

    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    print(devices)
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[type_camera]))
    camera.Open()
    if type_camera == 0:
        camera.Width = 2590
        camera.Height = 1942
    else:
        camera.Width = 3000
        camera.Height = 2000

    camera.CenterX.SetValue(True)
    camera.CenterY.SetValue(True)
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    # set_pin_state("ur", 5, True)

    if type_camera == 0:
        time.sleep(2)

    if camera.IsGrabbing():
        grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab.GrabSucceeded():
            img = pylon.PylonImage()
            img.AttachGrabResultBuffer(grab)
            filename = "src/ihm_tests/images/saved_pypylon_img_{}.png".format(type_camera)
            img.Save(pylon.ImageFileFormat_Png, filename)
            # set_pin_state("ur", 5, False)

            return filename
        camera.Close()



if __name__ == '__main__':
    print('ok')
    # camera_trigger_basler(1)
    # camera_basler(1)
