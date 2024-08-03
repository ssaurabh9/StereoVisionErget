from roughWrapper import *
from stereovision.stereo_cameras import *


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


