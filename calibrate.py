
# coding: utf-8
# Author: Francis (Github @heretic1993)
# License: MIT

import threading
import time
import rsAruco as ra
import cv2
from PIL import Image

import numpy as np
import matplotlib.pyplot as plt
from IPython import display
# Dobot manipulation
import DobotDllType as dType



thread1 = ra.cameraDetection(1, "rsArucoDetection")

def showImg():
    while True:
#         display.clear_output(wait=True)
        cv2.imwrite('./color.jpg', ra.color_image)
#         img=Image.open(r'./color.jpg')
#         plt.imshow(img)
    
thread2 = threading.Thread(target=showImg)



# start to find aruco code
if thread1.isAlive()==False:
    print("camera starts!")
    thread1.start()
    
time.sleep(2)
if thread2.isAlive()==False:
    print("showImg starts!")
    thread2.start()



#dobot init
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()


# Search 
# print(dType.SearchDobot(api))
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]

print("Connect status:",CON_STR[state])

#set home postion
dType.SetHOMEParams(api,217,0,154,0)

# reset to home position
# dType.SetHOMECmd(api, temp = 0, isQueued = 1)

# Calibration points
default_cali_points = [[180,-120,135,0],[270,-120,135,0],
                       [180,120,135,0],[270,120,135,0],
                       [270,120,-5,0],[180,120,-5,0],
                       [180,-120,-5,0],[270,-120,-5,0]]
np_cali_points = np.array(default_cali_points)
arm_cord = np.column_stack((np_cali_points[:,0:3], np.ones(np_cali_points.shape[0]).T)).T


# clear queue
dType.SetQueuedCmdClear(api)
dType.SetQueuedCmdStartExec(api)

centers=np.ones(arm_cord.shape)

for ind,pt in enumerate(default_cali_points):
    print("Current points:", pt)
#     dType.SetQueuedCmdStopExec(api)
#     dType.SetQueuedCmdClear(api)
    queuedCmdIndex = dType.SetPTPCmd(api, 1, pt[0], pt[1], pt[2], pt[3], isQueued=0);
    while dType.GetQueuedCmdCurrentIndex(api) != queuedCmdIndex:
        time.sleep(1)
    time.sleep(2)
    centers[0:3,ind]=ra.center
    print(ra.center)
    

image_to_arm = np.dot(arm_cord, np.linalg.pinv(centers))
arm_to_image = np.linalg.pinv(image_to_arm)
dType.SetPTPCmd(api, 1, 217,0,154,0, isQueued=0);
dType.SetQueuedCmdStopExec(api);

print("Finished")
print("Image to arm transform:\n", image_to_arm)
print("Arm to Image transform:\n", arm_to_image)
print("Sanity Test:")

print("-------------------")
print("Image_to_Arm")
print("-------------------")
for ind, pt in enumerate(centers.T):
    print("Expected:", default_cali_points[ind][0:3])
    print("Result:", np.dot(image_to_arm, np.array(pt))[0:3])
    
print("-------------------")
print("Arm_to_Image")
print("-------------------")
for ind, pt in enumerate(default_cali_points):
    print("Expected:", centers.T[ind][0:3])
    pt[3]=1
    print("Result:", np.dot(arm_to_image, np.array(pt))[0:3])

