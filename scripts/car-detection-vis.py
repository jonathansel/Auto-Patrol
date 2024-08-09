import rospy
from rosbag_append.msg import CarDetection, CarDetectionArray
from std_msgs.msg import Header

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import pcl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import math

import tf2_ros
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import time

def inverse_proj2(arr_inv, K_inv, coord):
    eps = 1e-24
    img_coord = coord
    world_coord = arr_inv @ K_inv @ img_coord
    world_coord[:2, :] = world_coord[:2, :] / (world_coord[2, :] + eps)
    world_coord[2, :] = 0
    return world_coord

def transform_coordinates(x, y, z, source_frame, target_frame, tf_buffer):

    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        
        # Extract the translation and rotation from the transformation
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Define a translation matrix
        translation_matrix = np.array([[1, 0, 0, trans.x],
                                        [0, 1, 0, trans.y],
                                        [0, 0, 1, trans.z],
                                        [0, 0, 0, 1]])
        
        # Define a rotation matrix using quaternion values
        q = [rot.x, rot.y, rot.z, rot.w]
        q_matrix = tf.transformations.quaternion_matrix(q)
        
        # Combine the transformations
        transformation_matrix = np.dot(translation_matrix, q_matrix)
        
        # Apply transformation
        point = np.array([x, y, z, 1])
        transformed_point = np.dot(transformation_matrix, point)[:3]
        
        print("Transformed coordinates: x=%f, y=%f, z=%f" % (transformed_point[0], transformed_point[1], transformed_point[2]))
        return transformed_point
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print("An error occurred during transformation:", e)
        return None
    
def publish_rectangle(p1, p2, p3, p4, rec_pub):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "rectangle"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.1  # Width of the line

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Alpha (opacity)

    # Add points to the marker
    marker.points = [p1, p2, p4, p3, p1]  # Complete the loop

    rec_pub.publish(marker)

def detection_callback(message, tf_buffer, arr_inv, K_inv, rec_pub):
    print("Received a car detection array")
    for det in message.detections:
        pix1 = np.array([[det.detection_vertices[0]], [det.detection_vertices[4]], [1]])
        pix2 = np.array([[det.detection_vertices[1]], [det.detection_vertices[5]], [1]])
        pix3 = np.array([[det.detection_vertices[2]], [det.detection_vertices[6]], [1]])
        pix4 = np.array([[det.detection_vertices[3]], [det.detection_vertices[7]], [1]])

        p1 = inverse_proj2(arr_inv, K_inv, pix1)
        p2 = inverse_proj2(arr_inv, K_inv, pix2)
        p3 = inverse_proj2(arr_inv, K_inv, pix3)
        p4 = inverse_proj2(arr_inv, K_inv, pix4)

        m1 = transform_coordinates(p1[0], p1[1], p1[2], 'base_link', 'map', tf_buffer)
        m2 = transform_coordinates(p2[0], p2[1], p2[2], 'base_link', 'map', tf_buffer)
        m3 = transform_coordinates(p3[0], p3[1], p3[2], 'base_link', 'map', tf_buffer)
        m4 = transform_coordinates(p4[0], p4[1], p4[2], 'base_link', 'map', tf_buffer)

        f1 = Point(m1[0], m1[1], m1[2])
        f2 = Point(m2[0], m2[1], m2[2])
        f3 = Point(m3[0], m3[1], m3[2])
        f4 = Point(m4[0], m4[1], m4[2])

        publish_rectangle(f1, f2, f3, f4, rec_pub)

class InverseProjector:
    def __init__(self):
        # extrinsic orientation of camera [rad] CHECK ORDER BEFORE WE ADD IN PITCH
        self.pitch = 0.11 #0.122173 
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
    
def main():
    rospy.init_node('car_detection_vis', anonymous=True)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer) 

    projector = InverseProjector()

    rec_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rospy.Subscriber("/car_detections", CarDetectionArray, lambda msg: detection_callback(msg, tf_buffer, projector.arr_inv, projector.K_inv, rec_pub))
      
    rospy.spin()

if __name__ == "__main__":
    main()