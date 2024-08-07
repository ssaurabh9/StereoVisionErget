import json
import math
import sys
import threading
import time
import pickle5 as pickle
# import utils
#import calibration
import cv2
sys.path.append("../MvImport")
from mvs.MvCameraControl_class import *
import datetime
g_bExit = False
#from tracker_object import FemureTracker, TiangleTracker, TibiaTracker, isosceles_triangle_coordinates
import numpy as np
from scipy.optimize import least_squares


def RightCameraInitialization(deviceList):
    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()
    nConnectionNum = 0  # For device based selections.
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()
    if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
        nPacketSize = cam.MV_CC_GetOptimalPacketSize()
        if int(nPacketSize) > 0:
            ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
            if ret != 0:
                print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
        else:
            print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)
    stBool = c_bool(False)

    ret = cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
    if ret != 0:
        print("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)
        sys.exit()
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nPayloadSize = stParam.nCurValue
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
    return cam, nPayloadSize


def LeftCameraInitialization(deviceList):
    cam = MvCamera()
    stDeviceList = cast(deviceList.pDeviceInfo[int(1)], POINTER(MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()
    if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
        nPacketSize = cam.MV_CC_GetOptimalPacketSize()
        if int(nPacketSize) > 0:
            ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
            if ret != 0:
                print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
        else:
            print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)
    stBool = c_bool(False)
    ret = cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
    if ret != 0:
        print("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)
        sys.exit()
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nPayloadSize = stParam.nCurValue
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
    return cam, nPayloadSize


def CameraCleanup(cam):
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        sys.exit()
    # ch:关闭设备 | Close device
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        sys.exit()
    # ch:销毁句柄 | Destroy handle
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        sys.exit()
    return


def RightCameraHandlePayload(cam, nPayloadSize, response=None):
    # ret = cam.MV_CC_StartGrabbing()
    # if ret != 0:
    #     print("start grabbing fail! ret[0x%x]" % ret)
    #     sys.exit()
    data_buf = byref((c_ubyte * nPayloadSize)())
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    ret = cam.MV_CC_GetOneFrameTimeout(data_buf, nPayloadSize, stFrameInfo, 500)
    if ret != 0:
        print("Error capturing frame from camera 1")
    frame_data = np.frombuffer(bytes(data_buf._obj), np.uint8).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 1))
    if response is not None and isinstance(response, list):
        response[0] = frame_data
    del data_buf
    return


def LeftCameraHandlePayload(cam, nPayloadSize, response=None):
    # ret = cam.MV_CC_StartGrabbing()
    # if ret != 0:
    #     print("start grabbing fail! ret[0x%x]" % ret)
    #     sys.exit()
    data_buf = byref((c_ubyte * nPayloadSize)())
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    ret = cam.MV_CC_GetOneFrameTimeout(data_buf, nPayloadSize, stFrameInfo, 500)
    if ret != 0:
        print("Error capturing frame from camera 1")
    frame_data = np.frombuffer(bytes(data_buf._obj), np.uint8).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 1))
    if response is not None and isinstance(response, list):
        response[1] = frame_data
    del data_buf
    return


def work_thread(cam_r, nPayloadSize_r, cam_l, nPayloadSize_l):
    results = [None, None]
    # detection_count = 0
    # right_detection_data=[]
    # left_detection_data=[]
    framecount = 0
    iteration_count = 0
    variable_dict = {}
    point_reg = ['HC']
    while True:

        thread_cam1 = threading.Thread(target=RightCameraHandlePayload, args=(cam_r, nPayloadSize_r, results))
        thread_cam2 = threading.Thread(target=LeftCameraHandlePayload, args=(cam_l, nPayloadSize_l, results))
        thread_cam1.start()
        thread_cam2.start()
        thread_cam1.join()
        thread_cam2.join()

        img_right, img_left = calibration.rectify_images(results[0], results[1])


        try:
            detections_right = utils.find_contours_Kmeans(img_right)
            detections_left = utils.find_contours_Kmeans(img_left)
            # detection_count = detection_count + 1
            if len(detections_right) == 6 and 6 == len(detections_left):
                detections_right = sorted(detections_right, key=lambda x: x[0])
                detections_left = sorted(detections_left, key=lambda x: x[0])
                # Extract first 3 coordinates
                first_three_right = detections_right[:3]
                first_three_left = detections_left[:3]
                # femure = FemureTracker(tuple(first_three_right), tuple(first_three_left), img_right, img_left)
                # femure.find_depth()
                # F_points = femure.getall3Dcordinates()
                # # F_points = femure.find_cordinate_triangulatin()
                # # print(f'F_points {F_points}')
                #
                detections_right = detections_right[3:6]
                detections_left = detections_left[3:6]
                next_three_right = sorted(detections_right, key=lambda detection: detection[0])
                next_three_left = sorted(detections_left, key=lambda detection: detection[0])
                femure = FemureTracker(tuple(next_three_right), tuple(next_three_left), img_right, img_left)
                femure.find_depth()
                F_points = femure.getall3Dcordinates()
                # continue
                # point_A = (F_points[1] + F_points[0]) / 2.0
                # vectorF = F_points[2] - point_A
                #
                # Femureangle = math.degrees(math.atan2(vectorF[1], vectorF[0]))
                # point_B = (T_Points[0] + T_Points[1]) / 2.0
                # vector = T_Points[2] - point_B

                # TibiaAngle = math.degrees(math.atan2(vector[1], vector[0]))
                # print(f'ROM Angle {180 - (TibiaAngle - Femureangle)}')
                # time.sleep(1)
                # continue

                # mid_point_femure = [(F_points[0][i] + F_points[1][i]) / 2 for i in range(3)]
                # femure_Axis_x = np.subtract(mid_point_femure, F_points[2])
                # femure_Axisy = np.subtract(F_points[0], F_points[1])
                # v1 = femur_Axis_z = np.cross(femure_Axis_x, femure_Axisy)
                #
                # mid_point_tibia = [(T_Points[0][i] + T_Points[1][i]) / 2 for i in range(3)]
                # v2 = T_Points_Axis_x = np.subtract(T_Points[2], mid_point_tibia)
                # T_Points_Axis_y = np.subtract(T_Points[0], T_Points[1])
                # T_Points_Axis_z = np.cross(T_Points_Axis_x, T_Points_Axis_y)
                #
                # def angle_between_vectors(vector1, vector2):
                #     # Convert vectors to numpy arrays
                #     vector1 = np.array(vector1)
                #     vector2 = np.array(vector2)
                #
                #     # Compute dot product
                #     dot_product = np.dot(vector1, vector2)
                #
                #     # Compute magnitudes
                #     vector1_magnitude = np.linalg.norm(vector1)
                #     vector2_magnitude = np.linalg.norm(vector2)
                #
                #     # Calculate cosine of the angle
                #     cos_theta = dot_product / (vector1_magnitude * vector2_magnitude)
                #
                #     # Use arccos to get the angle in radians
                #     angle_radians = np.arccos(np.clip(cos_theta, -1.0, 1.0))
                #
                #     # Convert radians to degrees
                #     angle_degrees = np.degrees(angle_radians)
                #
                #     return angle_degrees
                #
                # # print(f' f {mid_point_femure} t {mid_point_tibia} v1 {v1}  v2 {v2}')
                # angle = angle_between_vectors(v1, v2)
                # print(f'Angles {90 - angle}')
                #
                # continue

                Pointer_traker = TiangleTracker(tuple(first_three_right), tuple(first_three_left), img_right, img_left)
                Pointer_traker.find_depth()
                pointer_tip_gcs = Pointer_traker.getpointertip()

                femur_plane = [F_points[0], F_points[1], [0, 0, 0]]
                # Save new point and current femur properties
                variable_memory = {'C': pointer_tip_gcs, 'Plane': femur_plane}

                variable_dict[point_reg[0]] = variable_memory
                print(f'variable_dict {variable_dict}')
                # print(f'Point registered {point_reg[0]}')
                utils.play_notification_sound()
                # time.sleep(1)

                # try:
                #     with open(
                #             f'C:\\Users\Dell\\Desktop\\MK\\kneeSequence\\PrimaryRoboticKnee\\points\\{point_reg[framecount]}.json',
                #             'w') as f:
                #         json.dump(point_reg[framecount], f, indent=2)
                #         f.close()
                # except Exception as e:
                #     print(f"Error while openfile")
                time.sleep(1)
                framecount = framecount + 1
            # else:
            #     print('no data')
        except Exception as e:
            print(f'ERROR {e}')
            break
        if framecount == 10:
            # if framecount == 4:
            print("All points registered.")

            with open('HC_variables.pickle', 'wb') as handle:
                pickle.dump(variable_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

            print("Pickle dump successful")
            break
        if g_bExit == True:
            break


global cam_right, cam_left
try:
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()
    cam_right, nPayloadSize_right = RightCameraInitialization(deviceList)
    cam_left, nPayloadSize_left = LeftCameraInitialization(deviceList)
    work_thread(cam_right, nPayloadSize_right, cam_left, nPayloadSize_left)
except Exception as e:
    print(f'Error {e}')
finally:
    CameraCleanup(cam_right)
    CameraCleanup(cam_left)
