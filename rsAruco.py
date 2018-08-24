
# coding: utf-8
# Author: Francis (Github @heretic1993)
# License: MIT

import import_ipynb
import threading
import time

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import cv2.aruco as aruco
get_ipython().run_line_magic('matplotlib', 'inline')


center=np.array([1, 2, 3], dtype='float64')

class cameraDetection (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True
    def run(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        print("Start streaming")
        pipeline.start(config)
        ##########################
#         stream = pipeline.get_stream(rs.stream.depth)
#         intrinsics = stream.get_intrinsics()
        ##########################
        
   
        while cv2.waitKey(1)<0 and self.__running.isSet():
            global color_image
#             global color_image
            self.__flag.wait()      # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            # get intrinsic of color image
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
#             print(depth_image.shape)
#             print(color_image.shape)
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
#             images = np.hstack((color_image, depth_colormap))

            # Show images
        #         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #         cv2.imshow("RealSense", images)
        #         plt.figure(1)
        #         plt.subplot(121)
        #         plt.imshow(color_image)
        #         plt.subplot(122)
        #         plt.imshow(depth_image)
        #         cv2.waitKey(1)

            # Our operations on the frame come here
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            parameters =  aruco.DetectorParameters_create()

            #lists of ids and the corners beloning to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #         print(corners)
            if len(corners)!=0:
                point = np.average(corners[0][0], axis=0)
                depth = depth_frame.get_distance(point[0], point[1])
                point = np.append(point,depth)
                if depth!=0:
                    global center
#                     print("center:%f %f, depth:%f m" %(point[0], point[1], point[2]))
                    x=point[0]
                    y=point[1]
                    z=point[2]
                    ## see rs2 document: 
                    ## https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
                    ## and example: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
                    x,y,z=rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], z)
                    center=[x,y,z]
#                     print(center)
                    color_image = aruco.drawDetectedMarkers(color_image, corners)
                    
#                     cv2.imwrite('./color.jpg',color_image)

            #print(rejectedImgPoints)
            # Display the resulting frame
#             print("about to show!")
#             cv2.startWindowThread()
        ## if uncommented, crash!!!
#             cv2.namedWindow('Detection', cv2.WINDOW_AUTOSIZE)
#             cv2.imshow("Detection", color_image)
#             cv2.waitKey(1)


        # Stop streaming
        cv2.destroyAllWindows()
        pipeline.stop()
        time.sleep(1)
    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False  


# In[ ]:


def main():
    thread = cameraDetection(1, "rsArucoDetection")
    thread.start()
if __name__ == '__main__':
    main()

