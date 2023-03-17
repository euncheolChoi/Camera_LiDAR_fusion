# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rospy
from time import time
import time
from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from genpy.rostime import Time
from cv_bridge import CvBridge
import cv2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import sensor_msgs.point_cloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs


class DataSub:
    def __init__(self): 
        self.Image = rospy.Subscriber('/camera/color/image_rect_color', Image, self.callback_img)   # /camera/color/image_rect_color
        self.PointCloud2 = rospy.Subscriber('/velodyne_points', PointCloud2, self.callback_lidar)
        self.pub = rospy.Publisher('/custom_velodyne_points', PointCloud2, queue_size=10)
        self.camera_msg = None  #  먼저 초깃값 설정 -> DataSub object has no attribute called image_msg error 발생함
        self.lidar_msg = None

    def callback_img(self, image):
        self.camera_msg = image
        
        
    def callback_lidar(self, pointCloud):
        self.lidar_msg = pointCloud

    def pointcloud2_publish(self, xyz, bgr):      
        abgr = np.c_[bgr, np.zeros(shape=(bgr.shape[0], 1),dtype='u1')]     # u1
        pcPack = np.ndarray(buffer=abgr, dtype='f4', shape=(xyz.shape[0], 1))  # f4 -> pcPack shape (6989, 1) 

        header = Header()
        header.frame_id = 'velodyne'
        field = [pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
                 pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
                 pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
                 pc2.PointField("rgb", 12, pc2.PointField.FLOAT32, 1)
                ]
        points = np.c_[xyz, pcPack]  
        pc = point_cloud2.create_cloud(header, field, points)
    
        # print('bgr_limit dataType', bgr_limit.dtype)
        # print('abgr', abgr)
        # print('pcPack',pcPack) # 직렬화된 값
        # print('pcPack shape', pcPack.shape)

        return pc
            


class Dataload:
    def load_lidar(self, lidar_msg):
        pointcloud = ros_numpy.numpify(lidar_msg)
        points_non_homo = np.zeros((pointcloud.shape[0], 3))
        points_homo = np.zeros((pointcloud.shape[0], 4))    
        
        points_non_homo[:, 0] = pointcloud['x']
        points_non_homo[:, 1] = pointcloud['y']
        points_non_homo[:, 2] = pointcloud['z']

        points_homo[:, 0] = pointcloud['x']
        points_homo[:, 1] = pointcloud['y']
        points_homo[:, 2] = pointcloud['z']
        points_homo[:, 3] = pointcloud['x'] / pointcloud['x']  #  homo = 1

        return points_homo, points_non_homo


    def load_img(self, imgMsg):
        img = bridge.imgmsg_to_cv2(imgMsg, desired_encoding ="bgr8")
        b, g, r = cv2.split(img)
        
        # cv2.imshow('modified image', rgb_img)
        # cv2.waitKey(1)
        # cv2.imshow('original image', img)
        
        return img  

# ============= set parameter =============== #

def set_intrinsic_matrix():
    # image_raw
    # I = np.array([[602.7848, 0, 325.3909],
    #               [0, 602.5094, 248.8889],
    #               [0, 0, 1]])

    # rect_color
    I = np.array([[948.7858,  0,        631.7205],  # 631.7205
                  [0,         950.5275, 358.5715],  # 358.5715
                  [0,         0,               1]]) 


    # I = np.array([[940.7858,  0,        780],  # 631.7205
    #               [0,         942.5275, 300],  # 358.5715
    #               [0,         0,               1]])                 

    return I


def set_extrinsic_matrix():
    E = np.zeros((3, 4)) 


    # E의 역행렬을 계산한 결과(MRPT convention 기준)
    # E = np.array([[0.0040,     -0.9991,      -0.0415,     -0.0329667],         
    #               [0.0345,      0.0416,      -0.9985,     -0.29727165],        
    #               [0.9994,      0.0025,       0.0346,      0.03851164]])     
  
    # # rectified image로 E의 역행렬 계산한 결과
    E = np.array([[-0.0258,  -0.9994,  -0.0237,     -0.22725325],         
                  [0.038,     0.0227,  -0.999,     -0.07012617],        
                  [0.9989,   -0.0267,   0.0374,    0.16426952]])     


    return E

# ============== 좌표계 이동 =============== #

class CoordinateTransform:
    def world_to_camera_coor(homo_lidarPoints, E): 
        camera_arr = E.dot(np.transpose(homo_lidarPoints)) 
        return camera_arr       # 3 x 19745 vector 

    def camera_to_pixel(cameraArray, I):

        cameraArray = np.transpose(cameraArray)
        n_x = cameraArray[:, 0] / cameraArray[:, 2]  # Xc / Zc
        n_y = cameraArray[:, 1] / cameraArray[:, 2]  # Yc / Zc
        homo_z = cameraArray[:, 2] / cameraArray[:, 2]  # 1
        normal = np.column_stack((n_x, n_y, homo_z))  
        pixel_arr_no_limit = np.transpose(I.dot(np.transpose(normal))).astype(int)  # size*3

        return pixel_arr_no_limit 


def visualize_image(pixel, rgb_img):
    img_for_visualize = np.zeros((1280, 720, 3), np.uint8)
    for data in pixel:
        x = data[0]    
        y = data[1]

        if 0 < x < 1280 and 0 < y < 720:
            red_color = (0, 0, 255)
            img_for_visualize = cv2.circle(rgb_img, (x, y), 1, red_color, -1)
            
        
    cv2.imshow('image', img_for_visualize)    
    cv2.waitKey(1)    


def allocate_rgb_to_pixel(rgb_img, pixel_arr_no_limit):
    rgb_arr_size_limit = np.empty((1, 3))       # pixel 좌표를 480 * 640으로 제한하기 위해.
    xyz_filter_u = np.logical_and(pixel_arr_no_limit[:, 0] < 1280, pixel_arr_no_limit[:, 0] > 0)
    xyz_filter_v = np.logical_and(pixel_arr_no_limit[:, 1] < 720, pixel_arr_no_limit[:, 1] > 0)
    
    final_index = xyz_filter_u & xyz_filter_v
    uv_arr = pixel_arr_no_limit[final_index]
    u_arr = uv_arr[:, 0]
    v_arr = uv_arr[:, 1]
    
    rgb_arr_size_limit = rgb_img[v_arr, u_arr]
   
    # print('rgb_arr',rgb_img) # 4225, 3
    # print('rgb array size limit',rgb_arr_size_limit)
    # print('uv arr',uv_arr)
    # print('uv_arr shape', uv_arr.shape)
    # print('u_arr', u_arr.shape)
    
    return rgb_arr_size_limit, final_index


def select_final_xyz(index_vec, non_homo_lidarPoints):
    selected_xyz_point = np.empty((1, 3))
    for i, v in enumerate(index_vec):
        if v == 1 :
            camera_temp = np.array([non_homo_lidarPoints[i][:]])      
            selected_xyz_point = np.append(selected_xyz_point, camera_temp, axis = 0)
    selected_xyz_point = np.delete(selected_xyz_point, [0], axis = 0)
    print('selected xyz!!',selected_xyz_point.shape)
    
    return selected_xyz_point


# pixelArr와 rgbArr를 합쳐서 pointcloud2 Msg에 보내주기 위함
def make_xyz_rgb_point(xyzArr, rgbArr):
    # xyz_rgb = np.concatenate((xyzArr, rgbArr), axis = 1)
    # filtered_by_x = xyz_rgb[:, 0] > 0
    # xyz_rgb_final = xyz_rgb[filtered_by_x]
    
    filter = xyzArr[:, 0] > 0
    xyz = xyzArr[filter]
    rgb = rgbArr[filter]

    return xyz, rgb      



if __name__ == '__main__':
    rospy.init_node("camera_velodyne_subscriber")
    data = DataSub()
    #points = PointCloud2()
    dataload = Dataload() 
    bridge = CvBridge()
    transform = CoordinateTransform
    rospy.sleep(1)  # data 없을시 else문의 datatype None부분이 출력되므로.

    while not rospy.is_shutdown():
        start = time.time() 
        if data.lidar_msg != None and data.camera_msg != None:
            homo_lidarPoints, non_homo_lidarPoints = dataload.load_lidar(data.lidar_msg)
            rgb_img = dataload.load_img(data.camera_msg) 
            E = set_extrinsic_matrix()
            I = set_intrinsic_matrix()
            cameraArray = transform.world_to_camera_coor(homo_lidarPoints, E) 
            pixel_array_no_limit = transform.camera_to_pixel(cameraArray, I) 
            visualize_image(pixel_array_no_limit, rgb_img)
            rgb_size_limit, index = allocate_rgb_to_pixel(rgb_img, pixel_array_no_limit)     # 두 값 모두 (480 * 640)으로 크기가 제한된 값!
            
            final_xyz_point = select_final_xyz(index, non_homo_lidarPoints)  
            xyz, rgb = make_xyz_rgb_point(final_xyz_point, rgb_size_limit)
            # customPC = data.pointcloud2_publish(points)
            # customPC = data.pointcloud2_publish(final_xyz_point, rgb_size_limit)
            customPC = data.pointcloud2_publish(xyz, rgb)
            data.pub.publish(customPC)
            end = time.time()
            print('time : ',f"{end - start : .5f} sec")
        else:
            print('datatype None')