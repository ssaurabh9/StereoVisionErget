import os
import sys
import numpy as np
from os import getcwd
import cv2
import msvcrt
from ctypes import *
import threading

sys.path.append("../MvImport")
from mvs.MvCameraControl_class import *

import threading
import cv2
import os
from datetime import datetime
from ctypes import *

# methods created by saurabh for opencv compatibility
def capture_image(cam1, cam2, save_dir="captures"):
    os.makedirs(save_dir, exist_ok=True)
    cam1_index, cam2_index = 0, 0
    
    stOutFrame1 = MV_FRAME_OUT()
    stOutFrame2 = MV_FRAME_OUT()
    
    # Clear the structures
    memset(byref(stOutFrame1), 0, sizeof(stOutFrame1))
    memset(byref(stOutFrame2), 0, sizeof(stOutFrame2))

    ret1 = cam1.MV_CC_GetImageBuffer(stOutFrame1, 1000)
    ret2 = cam2.MV_CC_GetImageBuffer(stOutFrame2, 1000)

    if ret1 != 0 or ret2 != 0:
        print("Error: Failed to get image buffer")
        return

    # Handle image format conversion...
    image1 = convert_frame_to_image(stOutFrame1)
    image2 = convert_frame_to_image(stOutFrame2)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(os.path.join(save_dir, f"cam1_{timestamp}_{cam1_index}.png"), image1)
    cv2.imwrite(os.path.join(save_dir, f"cam2_{timestamp}_{cam2_index}.png"), image2)

    cam1_index += 1
    cam2_index += 1

    cv2.imshow("Camera 1", image1)
    cv2.imshow("Camera 2", image2)

    # Release resources
    cam1.MV_CC_FreeImageBuffer(stOutFrame1)
    cam2.MV_CC_FreeImageBuffer(stOutFrame2)

    # Check if the window is closed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        close_and_destroy_device(cam1)
        close_and_destroy_device(cam2)

def convert_frame_to_image(stOutFrame):
    if stOutFrame.stFrameInfo.enPixelType == 17301505:  # Mono8
        data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
        image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth)
    elif stOutFrame.stFrameInfo.enPixelType == 17301514:  # BayerGB8
        data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
        raw_image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth)
        image = cv2.cvtColor(raw_image, cv2.COLOR_BAYER_GB2RGB)
    elif stOutFrame.stFrameInfo.enPixelType == 35127316:  # RGB8
        data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)()
        cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)
        image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 3)
    elif stOutFrame.stFrameInfo.enPixelType == 34603039:  # YUV422
        data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
        cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
        raw_image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 2)
        image = cv2.cvtColor(raw_image, cv2.COLOR_YUV2BGR_Y422)
    else:
        raise ValueError("Unsupported pixel format")
    return image

def camera_feed(cam, window_name, stop_event):
    start_grab_and_get_data_size(cam)
    while not stop_event.is_set():
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if ret == 0:
            if stOutFrame.stFrameInfo.enPixelType == 17301505:  # Mono8
                data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth)
            elif stOutFrame.stFrameInfo.enPixelType == 17301514:  # BayerGB8
                data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                raw_image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth)
                image = cv2.cvtColor(raw_image, cv2.COLOR_BAYER_GB2RGB)
            elif stOutFrame.stFrameInfo.enPixelType == 35127316:  # RGB8
                data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)()
                cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)
                image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 3)
            elif stOutFrame.stFrameInfo.enPixelType == 34603039:  # YUV422
                data = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
                cdll.msvcrt.memcpy(byref(data), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
                raw_image = np.frombuffer(data, dtype=np.uint8).reshape(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 2)
                image = cv2.cvtColor(raw_image, cv2.COLOR_YUV2BGR_Y422)
            else:
                print("Unsupported pixel format.")
                continue

            # Resize the image
            resized_image = cv2.resize(image, (640, 480))
            cv2.imshow(window_name, resized_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
        else:
            print("Failed to get image buffer.")
        cam.MV_CC_FreeImageBuffer(stOutFrame)

    cv2.destroyWindow(window_name)
    close_and_destroy_device(cam)

def capture_frames(cam1, cam2, save_dir="captures"):
    start_grab_and_get_data_size(cam1)
    start_grab_and_get_data_size(cam2)

    stOutFrame1 = MV_FRAME_OUT()
    stOutFrame2 = MV_FRAME_OUT()
    memset(byref(stOutFrame1), 0, sizeof(stOutFrame1))
    memset(byref(stOutFrame2), 0, sizeof(stOutFrame2))

    ret1 = cam1.MV_CC_GetImageBuffer(stOutFrame1, 1000)
    ret2 = cam2.MV_CC_GetImageBuffer(stOutFrame2, 1000)

    if ret1 == 0 and ret2 == 0:
        if stOutFrame1.stFrameInfo.enPixelType == 17301505:  # Mono8
            data1 = (c_ubyte * stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(data1), stOutFrame1.pBufAddr, stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight)
            image1 = np.frombuffer(data1, dtype=np.uint8).reshape(stOutFrame1.stFrameInfo.nHeight, stOutFrame1.stFrameInfo.nWidth)
        elif stOutFrame1.stFrameInfo.enPixelType == 17301514:  # BayerGB8
            data1 = (c_ubyte * stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(data1), stOutFrame1.pBufAddr, stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight)
            raw_image1 = np.frombuffer(data1, dtype=np.uint8).reshape(stOutFrame1.stFrameInfo.nHeight, stOutFrame1.stFrameInfo.nWidth)
            image1 = cv2.cvtColor(raw_image1, cv2.COLOR_BAYER_GB2RGB)
        elif stOutFrame1.stFrameInfo.enPixelType == 35127316:  # RGB8
            data1 = (c_ubyte * stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight * 3)()
            cdll.msvcrt.memcpy(byref(data1), stOutFrame1.pBufAddr, stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight * 3)
            image1 = np.frombuffer(data1, dtype=np.uint8).reshape(stOutFrame1.stFrameInfo.nHeight, stOutFrame1.stFrameInfo.nWidth, 3)
        elif stOutFrame1.stFrameInfo.enPixelType == 34603039:  # YUV422
            data1 = (c_ubyte * stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight * 2)()
            cdll.msvcrt.memcpy(byref(data1), stOutFrame1.pBufAddr, stOutFrame1.stFrameInfo.nWidth * stOutFrame1.stFrameInfo.nHeight * 2)
            raw_image1 = np.frombuffer(data1, dtype=np.uint8).reshape(stOutFrame1.stFrameInfo.nHeight, stOutFrame1.stFrameInfo.nWidth, 2)
            image1 = cv2.cvtColor(raw_image1, cv2.COLOR_YUV2BGR_Y422)
        else:
            print("Unsupported pixel format for camera 1.")
            return None, None

        if stOutFrame2.stFrameInfo.enPixelType == 17301505:  # Mono8
            data2 = (c_ubyte * stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(data2), stOutFrame2.pBufAddr, stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight)
            image2 = np.frombuffer(data2, dtype=np.uint8).reshape(stOutFrame2.stFrameInfo.nHeight, stOutFrame2.stFrameInfo.nWidth)
        elif stOutFrame2.stFrameInfo.enPixelType == 17301514:  # BayerGB8
            data2 = (c_ubyte * stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(data2), stOutFrame2.pBufAddr, stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight)
            raw_image2 = np.frombuffer(data2, dtype=np.uint8).reshape(stOutFrame2.stFrameInfo.nHeight, stOutFrame2.stFrameInfo.nWidth)
            image2 = cv2.cvtColor(raw_image2, cv2.COLOR_BAYER_GB2RGB)
        elif stOutFrame2.stFrameInfo.enPixelType == 35127316:  # RGB8
            data2 = (c_ubyte * stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight * 3)()
            cdll.msvcrt.memcpy(byref(data2), stOutFrame2.pBufAddr, stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight * 3)
            image2 = np.frombuffer(data2, dtype=np.uint8).reshape(stOutFrame2.stFrameInfo.nHeight, stOutFrame2.stFrameInfo.nWidth, 3)
        elif stOutFrame2.stFrameInfo.enPixelType == 34603039:  # YUV422
            data2 = (c_ubyte * stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight * 2)()
            cdll.msvcrt.memcpy(byref(data2), stOutFrame2.pBufAddr, stOutFrame2.stFrameInfo.nWidth * stOutFrame2.stFrameInfo.nHeight * 2)
            raw_image2 = np.frombuffer(data2, dtype=np.uint8).reshape(stOutFrame2.stFrameInfo.nHeight, stOutFrame2.stFrameInfo.nWidth, 2)
            image2 = cv2.cvtColor(raw_image2, cv2.COLOR_YUV2BGR_Y422)
        else:
            print("Unsupported pixel format for camera 2.")
            return None, None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(os.path.join(save_dir, f"cam1_{timestamp}.png"), image1)
        cv2.imwrite(os.path.join(save_dir, f"cam2_{timestamp}.png"), image2)

        cam1.MV_CC_FreeImageBuffer(stOutFrame1)
        cam2.MV_CC_FreeImageBuffer(stOutFrame2)

        return image1, image2
    else:
        print("Failed to get image buffer.")
        return None, None



# Enumerate devices
def enum_devices(device=0, device_way=False):
    """
    device = 0: Enumerate GigE, USB, unknown devices, and Cameralink devices
    device = 1: Enumerate GenTL devices
    """
    if device_way == False:
        if device == 0:
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
            deviceList = MV_CC_DEVICE_INFO_LIST()
            # Enumerate devices
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print("Enum devices failed! ret[0x%x]" % ret)
                sys.exit()
            if deviceList.nDeviceNum == 0:
                print("No devices found!")
                sys.exit()
            print("Found %d devices!" % deviceList.nDeviceNum)
            return deviceList
        else:
            pass
    elif device_way == True:
        pass

# Identify different types of devices
def identify_different_devices(deviceList):
    # Identify different types of devices and print relevant information
    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        # Check if it is a GigE camera
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\nGigE device number: [%d]" % i)
            # Get device name
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("Device model name: %s" % strModeName)
            # Get current device IP address
            nip1_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip1_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip1_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip1_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("Current IP address: %d.%d.%d.%d" % (nip1_1, nip1_2, nip1_3, nip1_4))
            # Get current subnet mask
            nip2_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0xff000000) >> 24)
            nip2_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x00ff0000) >> 16)
            nip2_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x0000ff00) >> 8)
            nip2_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x000000ff)
            print("Current subnet mask: %d.%d.%d.%d" % (nip2_1, nip2_2, nip2_3, nip2_4))
            # Get current gateway
            nip3_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0xff000000) >> 24)
            nip3_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x00ff0000) >> 16)
            nip3_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x0000ff00) >> 8)
            nip3_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x000000ff)
            print("Current gateway: %d.%d.%d.%d" % (nip3_1, nip3_2, nip3_3, nip3_4))
            # Get network interface IP address
            nip4_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0xff000000) >> 24)
            nip4_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x00ff0000) >> 16)
            nip4_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x0000ff00) >> 8)
            nip4_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x000000ff)
            print("Connected network interface IP address: %d.%d.%d.%d" % (nip4_1, nip4_2, nip4_3, nip4_4))
            # Get manufacturer name
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("Manufacturer name: %s" % strmanufacturerName)
            # Get device version
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("Firmware version: %s" % stdeviceversion)
            # Get manufacturer-specific information
            stManufacturerSpecificInfo = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerSpecificInfo:
                stManufacturerSpecificInfo = stManufacturerSpecificInfo + chr(per)
            print("Manufacturer-specific information: %s" % stManufacturerSpecificInfo)
            # Get device serial number
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("Serial number: %s" % stSerialNumber)
            # Get user-defined name
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("User-defined name: %s" % stUserDefinedName)

        # Check if it is a USB camera
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nUSB device number: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("Device model name: %s" % strModeName)
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("Device serial number: %s" % strSerialNumber)
            # Get manufacturer name
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("Manufacturer name: %s" % strmanufacturerName)
            # Get device version
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("Firmware version: %s" % stdeviceversion)
            # Get device serial number
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("Serial number: %s" % stSerialNumber)
            # Get user-defined name
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("User-defined name: %s" % stUserDefinedName)
            # Get device GUID
            stDeviceGUID = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceGUID:
                stDeviceGUID = stDeviceGUID + chr(per)
            print("Device GUID: %s" % stDeviceGUID)
            # Get device family name
            stFamilyName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chFamilyName:
                stFamilyName = stFamilyName + chr(per)
            print("Device family name: %s" % stFamilyName)

        # Check if it is a 1394-a/b device
        elif mvcc_dev_info.nTLayerType == MV_1394_DEVICE:
            print("\n1394-a/b device: [%d]" % i)

        # Check if it is a Cameralink device
        elif mvcc_dev_info.nTLayerType == MV_CAMERALINK_DEVICE:
            print("\nCameralink device: [%d]" % i)
            # Get device name
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("Device model name: %s" % strModeName)
            # Get device serial number
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("Device serial number: %s" % strSerialNumber)
            # Get manufacturer name
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("Manufacturer name: %s" % strmanufacturerName)
            # Get device version
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("Firmware version: %s" % stdeviceversion)

# Input the number of the camera to connect
def input_num_camera(deviceList):
    nConnectionNum = input("Please input the number of the device to connect:")
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("Input error!")
        sys.exit()
    return nConnectionNum

# Create camera instance and handle (set log path)
def create_camera(deviceList, nConnectionNum, log=True, log_path=getcwd()):
    """
    :param deviceList:        Device list
    :param nConnectionNum:    Device number to connect
    :param log:               Create log or not
    :param log_path:          Log save path
    :return:                  Camera instance and device list
    """
    # Create camera instance
    cam = MvCamera()
    # Select device and create handle
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
    if log == True:
        ret = cam.MV_CC_SetSDKLogPath(log_path)
        print(log_path)
        if ret != 0:
            print("Set log path failed! ret[0x%x]" % ret)
            sys.exit()
        # Create handle, generate log
        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("Create handle failed! ret[0x%x]" % ret)
            sys.exit()
    elif log == False:
        # Create handle without log
        ret = cam.MV_CC_CreateHandleWithoutLog(stDeviceList)
        print(1111)
        if ret != 0:
            print("Create handle failed! ret[0x%x]" % ret)
            sys.exit()
    return cam, stDeviceList

# Open device
def open_device(cam):
    # Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("Open device failed! ret[0x%x]" % ret)
        sys.exit()

# Get various types of node parameters
def get_value(cam, param_type="int_value", node_name="PayloadSize"):
    """
    :param cam:            Camera instance
    :param_type:           Type of node value to get
    :param node_name:      Node name (can be int, float, enum, bool, or string type)
    :return:               Node value
    """
    if param_type == "int_value":
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = cam.MV_CC_GetIntValueEx(node_name, stParam)
        if ret != 0:
            print("Failed to get int type data %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        int_value = stParam.nCurValue
        return int_value

    elif param_type == "float_value":
        stFloatValue = MVCC_FLOATVALUE()
        memset(byref(stFloatValue), 0, sizeof(MVCC_FLOATVALUE))
        ret = cam.MV_CC_GetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("Failed to get float type data %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        float_value = stFloatValue.fCurValue
        return float_value

    elif param_type == "enum_value":
        stEnumValue = MVCC_ENUMVALUE()
        memset(byref(stEnumValue), 0, sizeof(MVCC_ENUMVALUE))
        ret = cam.MV_CC_GetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("Failed to get enum type data %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        enum_value = stEnumValue.nCurValue
        return enum_value

    elif param_type == "bool_value":
        stBool = c_bool(False)
        ret = cam.MV_CC_GetBoolValue(node_name, stBool)
        if ret != 0:
            print("Failed to get bool type data %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        return stBool.value

    elif param_type == "string_value":
        stStringValue = MVCC_STRINGVALUE()
        memset(byref(stStringValue), 0, sizeof(MVCC_STRINGVALUE))
        ret = cam.MV_CC_GetStringValue(node_name, stStringValue)
        if ret != 0:
            print("Failed to get string type data %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        string_value = stStringValue.chCurValue
        return string_value

# Set various types of node parameters
def set_value(cam, param_type="int_value", node_name="PayloadSize", node_value=None):
    """
    :param cam:               Camera instance
    :param param_type:        Type of node value to set
        int:
        float:
        enum:     Refer to Enum Entry Value in the client for this option
        bool:     Corresponds to 0 for off, 1 for on
        string:   The input value must be a number or English characters, cannot be Chinese characters
    :param node_name:         Name of the node to set
    :param node_value:        Value to set for the node
    :return:
    """
    if param_type == "int_value":
        stParam = int(node_value)
        ret = cam.MV_CC_SetIntValueEx(node_name, stParam)
        if ret != 0:
            print("Failed to set int type data node %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("Successfully set int type data node %s! Set value: %s!" % (node_name, node_value))

    elif param_type == "float_value":
        stFloatValue = float(node_value)
        ret = cam.MV_CC_SetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("Failed to set float type data node %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("Successfully set float type data node %s! Set value: %s!" % (node_name, node_value))

    elif param_type == "enum_value":
        stEnumValue = node_value
        ret = cam.MV_CC_SetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("Failed to set enum type data node %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("Successfully set enum type data node %s! Set value: %s!" % (node_name, node_value))

    elif param_type == "bool_value":
        ret = cam.MV_CC_SetBoolValue(node_name, node_value)
        if ret != 0:
            print("Failed to set bool type data node %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("Successfully set bool type data node %s! Set value: %s!" % (node_name, node_value))

    elif param_type == "string_value":
        stStringValue = str(node_value)
        ret = cam.MV_CC_SetStringValue(node_name, stStringValue)
        if ret != 0:
            print("Failed to set string type data node %s! Error code ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("Successfully set string type data node %s! Set value: %s!" % (node_name, node_value))

# Read or write memory
def read_or_write_memory(cam, way="read"):
    if way == "read":
        pass
        cam.MV_CC_ReadMemory()
    elif way == "write":
        pass
        cam.MV_CC_WriteMemory()

# Check if the camera is online
def decide_device_on_line(cam):
    value = cam.MV_CC_IsDeviceConnected()
    if value == True:
        print("The device is online!")
    else:
        print("The device is offline!", value)

# Set the number of internal image buffer nodes in the SDK
def set_image_node_num(cam, Num=1):
    ret = cam.MV_CC_SetImageNodeNum(nNum=Num)
    if ret != 0:
        print("Failed to set the number of internal image buffer nodes in the SDK, error code ret[0x%x]" % ret)
    else:
        print("Successfully set the number of internal image buffer nodes in the SDK to %d!" % Num)

# Set the grab strategy
def set_grab_strategy(cam, grabstrategy=0, outputqueuesize=1):
    """
    • OneByOne: Get images from the output buffer list one by one from old to new. This is the default strategy after the device is opened.
    • LatestImagesOnly: Get only the latest frame from the output buffer list and clear the output buffer list.
    • LatestImages: Get the latest OutputQueueSize frames from the output buffer list. The OutputQueueSize range is 1 - ImageNodeNum, which can be set using the MV_CC_SetOutputQueueSize() interface. The default value for ImageNodeNum is 1, which can be set using the MV_CC_SetImageNodeNum() interface. Setting OutputQueueSize to 1 is equivalent to the LatestImagesOnly strategy, and setting OutputQueueSize to ImageNodeNum is equivalent to the OneByOne strategy.
    • UpcomingImage: Ignore all images in the output buffer list and wait for the device to generate the next frame when the grab interface is called. This strategy only supports GigE devices and does not support U3V devices.
    """
    if grabstrategy != 2:
        ret = cam.MV_CC_SetGrabStrategy(enGrabStrategy=grabstrategy)
        if ret != 0:
            print("Failed to set grab strategy, error code ret[0x%x]" % ret)
        else:
            print("Successfully set grab strategy to %d!" % grabstrategy)
    else:
        ret = cam.MV_CC_SetGrabStrategy(enGrabStrategy=grabstrategy)
        if ret != 0:
            print("Failed to set grab strategy, error code ret[0x%x]" % ret)
        else:
            print("Successfully set grab strategy to %d!" % grabstrategy)

        ret = cam.MV_CC_SetOutputQueueSize(nOutputQueueSize=outputqueuesize)
        if ret != 0:
            print("Failed to set output queue size, error code ret[0x%x]" % ret)
        else:
            print("Successfully set output queue size to %d!" % outputqueuesize)

# Show image
def image_show(image, name):
    image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)
    name = str(name)
    cv2.imshow(name, image)
    cv2.imwrite("name.bmp", image)
    k = cv2.waitKey(1) & 0xff

# Convert the image data to be displayed
def image_control(data, stFrameInfo):
    if stFrameInfo.enPixelType == 17301505:
        image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
        image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 17301514:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_BAYER_GB2RGB)
        image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 35127316:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 34603039:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
        image_show(image=image, name=stFrameInfo.nHeight)

# Active image acquisition
def access_get_image(cam, active_way="getImagebuffer"):
    """
    :param cam:     Camera instance
    :active_way:    Different methods of active image acquisition (getImagebuffer or getoneframetimeout)
    :return:
    """
    if active_way == "getImagebuffer":
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        while True:
            ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            if None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301505:
                print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight), dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301514:
                print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight), dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 35127316:
                print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3), dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 34603039:
                print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2), dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            else:
                print("No data [0x%x]" % ret)
            nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)

    elif active_way == "getoneframetimeout":
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
        if ret != 0:
            print("Get payload size failed! ret[0x%x]" % ret)
            sys.exit()
        nDataSize = stParam.nCurValue
        pData = (c_ubyte * nDataSize)()
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
            if ret == 0:
                print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                image = np.asarray(pData)
                image_control(data=image, stFrameInfo=stFrameInfo)
            else:
                print("No data [0x%x]" % ret)

# Callback image acquisition
winfun_ctype = WINFUNCTYPE
stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
pData = POINTER(c_ubyte)
FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)

def image_callback(pData, pFrameInfo, pUser):
    global img_buff
    img_buff = None
    stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
    if stFrameInfo:
        print("Get one frame: Width[%d], Height[%d], FrameNum[%d]" % (stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
    if img_buff is None and stFrameInfo.enPixelType == 17301505:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 17301514:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 35127316:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 3)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 3)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 3), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 34603039:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 2)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 2)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 2), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff

CALL_BACK_FUN = FrameInfoCallBack(image_callback)

# Event callback
stEventInfo = POINTER(MV_EVENT_OUT_INFO)
pData = POINTER(c_ubyte)
EventInfoCallBack = winfun_ctype(None, stEventInfo, c_void_p)

def event_callback(pEventInfo, pUser):
    stPEventInfo = cast(pEventInfo, POINTER(MV_EVENT_OUT_INFO)).contents
    nBlockId = stPEventInfo.nBlockIdHigh
    nBlockId = (nBlockId << 32) + stPEventInfo.nBlockIdLow
    nTimestamp = stPEventInfo.nTimestampHigh
    nTimestamp = (nTimestamp << 32) + stPEventInfo.nTimestampLow
    if stPEventInfo:
        print("EventName[%s], EventId[%u], BlockId[%d], Timestamp[%d]" % (stPEventInfo.EventName, stPEventInfo.nEventID, nBlockId, nTimestamp))

CALL_BACK_FUN_2 = EventInfoCallBack(event_callback)

# Register callback for image acquisition
def call_back_get_image(cam):
    # Register image callback
    ret = cam.MV_CC_RegisterImageCallBackEx(CALL_BACK_FUN, None)
    if ret != 0:
        print("Register image callback failed! ret[0x%x]" % ret)
        sys.exit()

# Close device and destroy handle
def close_and_destroy_device(cam, data_buf=None):
    # Stop grabbing
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("Stop grabbing failed! ret[0x%x]" % ret)
        sys.exit()
    # Close device
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("Close device failed! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    # Destroy handle
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("Destroy handle failed! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    del data_buf

# Start grabbing and get data packet size
def start_grab_and_get_data_size(cam):
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("Start grabbing failed! ret[0x%x]" % ret)
        sys.exit()

def main():
    # Enumerate devices
    deviceList = enum_devices(device=0, device_way=False)
    # Identify different types of devices
    # identify_different_devices(deviceList)
    
    # Ensure there are at least two devices
    if deviceList.nDeviceNum < 2:
        print("Need at least two devices to display feeds from both cameras.")
        sys.exit()
    
    # Create camera instances and handles
    cam1, stDeviceList1 = create_camera(deviceList, 0, log=False)
    cam2, stDeviceList2 = create_camera(deviceList, 1, log=False)

    # Open devices
    open_device(cam1)
    open_device(cam2)
    
    capture_frames(cam1, cam2)
    

    # # Create a stop event
    # stop_event = threading.Event()
    
    # # Start threads for each camera feed
    # thread1 = threading.Thread(target=camera_feed, args=(cam1, "Camera 1", stop_event))
    # thread2 = threading.Thread(target=camera_feed, args=(cam2, "Camera 2", stop_event))

    # thread1.start()
    # thread2.start()

    # thread1.join()
    # thread2.join()

if __name__ == "__main__":
    main()
