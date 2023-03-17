# -*- coding: utf-8 -*-
#!/usr/bin/env python

import os
import rospy
import keyboard
from time import time
import time
from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from genpy.rostime import Time
from cv_bridge import CvBridge
import cv2
import pcl 
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np


class DataSub:

    def __init__(self):
        self.Image = rospy.Subscriber('/camera/color/image_rect_color', Image, self.callback_img)
        self.PointCloud2 = rospy.Subscriber('/velodyne_points', PointCloud2, self.callback_lidar)
        self.camera_msg = None
        self.lidar_msg = None
        

    def callback_img(self, camera_msg):
        self.camera_msg = camera_msg
        
        
    def callback_lidar(self, lidar_msg):
        self.lidar_msg = lidar_msg

# class for saving data, select, delete.
class DataProcess:  
    def __init__(self):
        pass


    def save(self, imgMsg, pointCloudMsg):  
        max_time_end = time.time() + 0.01    # get msg during 1sec 
        while True:  
            if time.time() > max_time_end:
                savedImgMsg.append(imgMsg)    
                savedPointMsg.append(pointCloudMsg) 
                break
        rospy.sleep(0.3)   
        
        """
        rospy.sleep()
        중요 : camera msg및 LiDAR msg가 각각 20hz, 10hz인데, sleep() 없이 save메서드를 10번 동작시킨다고 하면, 프로그램 실행 시간이 sensor msg input 시간보다 훨씬
        작기 때문에 같은 timestamp를 가진 msg를 10개 얻게 된다.
        따라서 다른 timestamp를 가진 msg data를 취득하기 위해서는 위와 같이 save 메서드 실행 사이에 sleep을 줘야 한다.

        """


    # selection 
    def selectAndDelete(self, savedImgMsgList, savedPointMsgList):    
        diffList = []
        diff_min = 987654321
        min_index = 0
        for i in range(10):  
            img_timestamp = savedImgMsgList[i].header.stamp.to_nsec()
            point_timeStamp = savedPointMsgList[i].header.stamp.to_nsec()
            timeStampDiff = abs(img_timestamp - point_timeStamp)
            diffList.append(timeStampDiff)
            print('timeStampDiff is : ',timeStampDiff)
            if diff_min > diffList[i]:       # 가장 작은 값을 비교하여 삽입
                diff_min = diffList[i] #timeStampDiff
                min_index = i
                print(min_index)
        print('---------------------','min_index is : ',min_index,'---------------------')

        selectedImgMsgList.append(savedImgMsgList[min_index])
        selectedPointMsgList.append(savedPointMsgList[min_index])
        
        savedImgMsgList.clear()
        savedPointMsgList.clear()
        diffList.clear()
        rospy.sleep(0.5)

        
    # 먼저 OpenCV 이용해서 .jpeg로 변환하고 파일 경로에 저장 
    def convertAndSave(self, selectedImgMsgList, seletedPointMsgList):
        cvImgList = []
        pointcloudList = []
        imageFilePath = '/home/cheol/calibration_ws/converted_data/img/0.jpeg'
        pointFilePath = '/home/cheol/calibration_ws/converted_data/pc/0.pcd'

        # imwrite의 경우 동일한 파일 이름으로 계속 덮어쓰기 때문에, 이름을 iter마다 바꿔줘야 함
        # 이미지 변환 및 저장
        for i in range(len(selectedImgMsgList)):
            if os.path.exists('/home/cheol/calibration_ws/converted_data/img/0.jpeg'):
                imageFilePath = '/home/cheol/calibration_ws/converted_data/img/' + str(i) + '.jpeg'
                           
            cvImgList.append(bridge.imgmsg_to_cv2(selectedImgMsgList[i], desired_encoding='bgr8'))    
            cv2.imwrite(imageFilePath, cvImgList[i])
            cv2.imshow("Image Window",cvImgList[i])
            cv2.waitKey(5)
            cv2.destroyAllWindows()
        # 포인트클라우드 변환 및 저장
        for j in range(len(selectedPointMsgList)):
             if os.path.exists('/home/cheol/calibration_ws/converted_data/img/0.jpeg'):
                pointFilePath = '/home/cheol/calibration_ws/converted_data/pc/' + str(j) + '.pcd'

                # 포인트클라우드 변환
                pointcloudList.append(ros_numpy.numpify(seletedPointMsgList[j]))
                #print(pointcloudList[i])
                points = np.zeros((pointcloudList[j].shape[0], 3))
                pc = pointcloudList[j]
                points[:,0]=pc['x']
                points[:,1]=pc['y']
                points[:,2]=pc['z']
                finalPoint = pcl.PointCloud(np.array(points, dtype=np.float32))

                # 포인트클라우드 저장
                pcl.save(finalPoint, pointFilePath)
        rospy.sleep(0.5) 
                
class KeyboardAction:
    def __init__(self):
        pass
    
    def action_enter(self):
        if keyboard.is_pressed('enter'):
            print('입력 : 엔터 키(save)')  
            print('wait for 5 seconds..')
            for i in range(10):     # data 10개씩 저장
                process.save(data.camera_msg, data.lidar_msg)
            
    def action_select(self):
        if keyboard.is_pressed('s'):
            print('\n')
            print('입력 : S 키(select)')
            process.selectAndDelete(savedImgMsg, savedPointMsg)
        
     
    def action_convert(self):
        if keyboard.is_pressed('c'):
            print('\n')
            print('입력 : C 키(convert)')
            print('Message converting finished, Press Enter.')
            process.convertAndSave(selectedImgMsgList, selectedPointMsgList)

if __name__ == '__main__':
    rospy.init_node("camera_velodyne_subscriber")
    points = PointCloud2()
    bridge = CvBridge()
    savedImgMsg = []
    savedPointMsg = []
    selectedImgMsgList = []
    selectedPointMsgList = []   
    finalDiffList = []

    data = DataSub()
    mykey = KeyboardAction()
    process = DataProcess()

    time.sleep(0.03)

    while not rospy.is_shutdown():
        mykey.action_enter()
        mykey.action_select()
        mykey.action_convert()