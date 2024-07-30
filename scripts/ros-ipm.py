import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import copy
import open3d as o3d
from tqdm import tqdm
from createPointCloud2 import create_point_cloud2
from numba import jit

import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

# this follows the publication but finding the inverse of A in runtime is comp heavy - takes 3 seconds after jit
@jit(nopython=True)
def inverse_projection(P, t, img_x, img_y):
    '''Transforms a point from 'image coordinates' (x_I, y_I) [px] -> 'world (plane) coordinates' (x_W, y_W, z_W) [m] where z = 0
    
    Args:
        img_x: Image 'x' coordinate [px]
        img_y: Image 'y' coordinate [px]
        P: Rotation matrix (world -> image coordinates)
        t: Translation vector (world -> image coordinates)
        
    Returns:
        World coordinate vector (x_W, y_W, z_W) [m] representing road plane location of image coordinate (x_I, y_I)
    '''
    
    # Inverted matrix
    A = np.zeros((4,4))
    A[0:3,0:3] = P
    A[0, 3] = -img_x
    A[1, 3] = -img_y
    A[2, 3] = -1
    A[3,2] = 1

    A_inv = np.linalg.inv(A)

    # Column vector
    t_vec = np.zeros((4,1))
    t_vec[0:3,:] = -t
    
    world_coord = A_inv @ t_vec
    return world_coord[:3]

# this follows https://www.youtube.com/watch?v=DPRFZODRf9E&t=515s&ab_channel=T%C3%BCbingenMachineLearning which is much simpler and more intuitive to me takes 1s less
# @jit(nopython=True) didn't work..
def inverse_proj2(arr_inv, K_inv, coord):
    eps = 1e-24
    img_coord = coord
    world_coord = arr_inv @ K_inv @ img_coord
    world_coord[:2, :] = world_coord[:2, :] / (world_coord[2, :] + eps)
    world_coord[2, :] = 0
    return world_coord

class InverseProjector:
    def __init__(self, camera_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(camera_topic, msg_Image, self.imageCallback)
        self.point_cloud_pub = rospy.Publisher("point_cloud_topic", PointCloud2, queue_size=10)

        # extrinsic orientation of camera [rad] CHECK ORDER BEFORE WE ADD IN PITCH
        self.pitch = 0.122173
        self.roll = 0.0
        self.yaw = 1.57  # Note about the rotation order and its impact

        # extrinsic position of camera [m] 
        self.x = -0.2489
        self.y = 0.18991
        self.z = 1.05653         #0.81653 (from base link) + wheel radius (to base_footprint) works well!

        # intrinsic parameters of camera [px]
        self.fx = 901.1387623297165
        self.fy = 902.8926336853814
        self.u0 = 645.2704047048309
        self.v0 = 366.13035318739895

        # Intrinsic parameter matrix
        self.K = np.array([[self.fx, 0, self.u0],
                           [0, self.fy, self.v0],
                           [0,  0,  1]])

        self.dist_coeffs = np.array([0.07968248, -0.13313626, 0.00236398, -0.00306793]) 

        # Compute rotation and translation matrices
        self.R_c2w = self.rotation_matrix()
        self.T_c2w = np.array([[self.x, self.y, self.z]]).T

        self.R_w2c = self.R_c2w.T
        self.T_w2c = -(self.R_w2c @ self.T_c2w) # note how its not simply the inverse

        self.R_cam2img = np.array([[0, -1, 0],
                                   [0, 0, -1],
                                   [1, 0,  0]])

        self.C = self.K @ self.R_cam2img
        self.P = self.C @ self.R_w2c 
        self.t = self.C @ self.T_w2c

        ## For inverseProj2 - TODO change to meaningful names
        self.arr = np.zeros((3,3))
        self.R_w2i = self.R_cam2img @ self.R_w2c
        self.T_w2i = (self.R_cam2img @ self.T_w2c) 

        self.arr[:, 0] = self.R_w2i[:,0] #rx
        self.arr[:, 1] = self.R_w2i[:,1] #ry
        self.arr[:, 2:3] = self.T_w2i
        # apparently this is a homography matrix

        self.arr_inv = np.linalg.inv(self.arr) 
        self.K_inv = np.linalg.inv(self.K)


    def rotation_matrix(self):
        """
        """
        roll, pitch, yaw = self.roll, self.pitch, self.yaw
        si, sj, sk = np.sin(roll), np.sin(pitch), np.sin(yaw)
        ci, cj, ck = np.cos(roll), np.cos(pitch), np.cos(yaw)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk

        R = np.identity(3)
        R[0, 0] = cj * ck
        R[0, 1] = sj * sc - cs
        R[0, 2] = sj * cc + ss
        R[1, 0] = cj * sk
        R[1, 1] = sj * ss + cc
        R[1, 2] = sj * cs - sc
        R[2, 0] = -sj
        R[2, 1] = cj * si
        R[2, 2] = cj * ci
        return R

    def imageCallback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding="mono8")    
            # IMG_H, IMG_W = img.shape # 720 1280 3 

            #### Undisort images, seems to worsen things. ###
            # h, w = img.shape[:2]
            # newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.dist_coeffs, (w,h), 1, (w,h))
            # img = cv.undistort(img, self.K, self.dist_coeffs, None, newcameramtx)

            # Find indices of pixels that have a value of 255
            y_indices, x_indices = np.where(img == 255)
            z_indices = np.ones_like(x_indices)

            # Stack x_indices and y_indices to get a (3, n) shape array
            coordinates = np.vstack((x_indices, y_indices, z_indices))

            real_world_position = inverse_proj2(self.arr_inv, self.K_inv, coordinates) #produce (3,n)
            points_with_colors = []

            # for y in tqdm(range(IMG_H)):
            #     for x in range(IMG_W):
            #         # Apply inverse projection to get 3D coordinates
            #         # Assuming inverse_projection_function is defined elsewhere
            #         # real_world_position = inverse_projection(self.P, self.t, x, y)
            #         real_world_position = inverse_proj2(self.arr_inv, self.K_inv, x, y)

            #         # Get RGB values
            #         # OpenCV loads an image in BGR format
            #         b, g, r = img[y, x]
                    
            #         # real_world_position is a numpy array with shape (3, 1)
            #         real_world_position_tuple = tuple(real_world_position.flatten())
            #         points_with_colors.append(real_world_position_tuple + (r, g, b))

            # this is like upper limit, if need a square zone, add a lower limit and an & in if statement
            compare_values = np.array([[10.0], [10.0], [1.0]]) # surprised it is that far..
            compare_values_flattened = compare_values.flatten()

            # print(real_world_position[:, 2].shape)

            # print(real_world_position[:, 2])
            for i in range(len(x_indices)):
                # x, y = x_indices[i], y_indices[i]
                # Extract RGB values directly using indices (OpenCV uses BGR order)
                b, g, r = 255, 255, 255
                # print((real_world_position[:, i] <= compare_values).all)
                # Construct the tuple and append it to the list
                if (real_world_position[:, i] <= compare_values_flattened).all():
                    real_world_position_tuple = tuple(real_world_position[:, i])  # Convert column to tuple
                    points_with_colors.append(real_world_position_tuple + (r, g, b))

            # create_point_cloud2 is a function that creates a PointCloud2 message
            point_cloud2_msg = create_point_cloud2(points_with_colors)

            # Publish the message
            self.point_cloud_pub.publish(point_cloud2_msg)

        except CvBridgeError as e:
            print(e)
            return
    
def main():
    camera_topic = '/cam1/lane_mask'
    projector = InverseProjector(camera_topic)
    rospy.spin() 

if __name__ == '__main__':
    rospy.init_node('inversePerspective', anonymous=True)
    main()