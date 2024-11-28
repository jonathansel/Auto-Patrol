import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import pcl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import math

from createPointCloud2 import create_point_cloud2, pcl_to_ros, pcl_to_ros_with_color
import struct
from std_msgs.msg import Header

import tf2_ros
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import time

# rospy.sleep(1)  # Allow some time for the buffer to fill

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
    
def point_cloud2_to_pcl(point_cloud2_msg):
    """
    Convert a ROS PointCloud2 message to a PCL Point Cloud.
    """
    cloud_points = list(pc2.read_points(point_cloud2_msg, skip_nans=True, field_names=("x", "y", "z")))
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(cloud_points)
    return pcl_cloud

def ransac_line_detection(pcl_cloud):
    """
    Use RANSAC to detect lines in the PCL Point Cloud.
    """
    start_time = time.perf_counter()

    # Statistical Outlier Removal
    sor = pcl_cloud.make_statistical_outlier_filter()
    sor.set_mean_k(60) #higher = more processing time
    sor.set_std_dev_mul_thresh(0.25)
    cleaned_cloud = sor.filter()

    seg = cleaned_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_LINE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.06)  # Was 0.1 - Set the distance threshold between points, points within this are considered inliers

    indices, coefficients = seg.segment()
    inlier_object = pcl_cloud.extract(indices, negative=False)
    outlier_object = pcl_cloud.extract(indices, negative=True)
    end_time = time.perf_counter()
    print(f"Detected RANSAC in {end_time - start_time:.6f} seconds")

    return inlier_object, coefficients, outlier_object

def publish_line_marker(inlier_object, coefficients, id, rcolor, gcolor, publisher, tf_buffer, frame_id="map"):
    """
    Publishes a line marker to visualize in RViz.
    """
    marker = Marker()
    marker.id = id
    marker.header.frame_id = frame_id
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = 0.1  # Line width
    marker.color.a = 1.0  # Opacity
    marker.color.r = rcolor
    marker.color.g = gcolor
    marker.color.b = 0.0

    inlier_points = np.array(inlier_object)
    point_on_line = np.array([coefficients[0], coefficients[1], coefficients[2]])
    direction_vector = np.array([coefficients[3], coefficients[4], coefficients[5]])  # Assumed to be normalized

    # Project points onto the line
    # Compute the vector from the point on the line to each inlier point
    vectors = inlier_points - point_on_line

    # Compute dot products in a vectorized manner
    dot_products = np.dot(vectors, direction_vector)

    # Compute the projections
    projections = np.outer(dot_products, direction_vector) + point_on_line

    # Find extremities
    min_projection_idx = np.argmin(dot_products)
    max_projection_idx = np.argmax(dot_products)

    min_projection_point = projections[min_projection_idx]
    max_projection_point = projections[max_projection_idx]
      
    # Start and end points of the line in base_link
    # p1 = Point(coefficients[0], coefficients[1], coefficients[2])
    # p2 = Point(coefficients[0] + coefficients[3], coefficients[1] + coefficients[4], coefficients[5])

    ## Transform to world coordinates - need error handling for if None type returned ##
    transform_p1 = transform_coordinates(min_projection_point[0], min_projection_point[1], min_projection_point[2], 'base_link', 'map', tf_buffer)
    transform_p2 = transform_coordinates(max_projection_point[0], max_projection_point[1], max_projection_point[2], 'base_link', 'map', tf_buffer)

    # p1 = Point(min_projection_point[0], min_projection_point[1], min_projection_point[2])
    # p2 = Point(max_projection_point[0], max_projection_point[1], max_projection_point[2])

    p1 = Point(transform_p1[0], transform_p1[1], transform_p1[2]) # min projection - but apparently furthest point
    p2 = Point(transform_p2[0], transform_p2[1], transform_p2[2]) # max projection - closest point

    ### Line Length Calculation ###
    # euclid_distance = math.sqrt((max_projection_point[0] - min_projection_point[0])**2 + (max_projection_point[1] - min_projection_point[1])**2)
    # print(euclid_distance)

    marker.points.append(p1)
    marker.points.append(p2)

    # Publish the marker
    publisher.publish(marker)

def cloud_callback(point_cloud2_msg, line1_pub, line2_pub, outlier_pub, tf_buffer):
    """
    Callback function for PointCloud2 subscriber.
    """
    pcl_cloud = point_cloud2_to_pcl(point_cloud2_msg)
    inlier_object, coefficients, outlier_objects = ransac_line_detection(pcl_cloud)

    outlier_ros_cloud = pcl_to_ros_with_color(inlier_object, 255, 0 , 0)
    outlier_pub.publish(outlier_ros_cloud)

    ## First Detected Line
    if coefficients:
        publish_line_marker(inlier_object, coefficients, 1, 0, 1, line1_pub, tf_buffer)
        print(f"Published line marker line 1: {coefficients}")

    ## Second Detected Line
    if outlier_objects.size > 0:
        ## For debugging outlier pointcloud
        # outlier_ros_cloud = pcl_to_ros_with_color(inlier_object, 255, 0 , 0)
        # pub.publish(outlier_ros_cloud)
        inlier_object, coefficients, outlier_objects = ransac_line_detection(outlier_objects)

    if coefficients:
        publish_line_marker(inlier_object, coefficients, 2, 1, 0, line2_pub, tf_buffer)
        print(f"Published line marker line 2: {coefficients}")

    ## Third Detected Line
    # if outlier_objects.size > 0:
    #     ## For debugging outlier pointcloud
    #     # outlier_ros_cloud = pcl_to_ros_with_color(inlier_object, 255, 0 , 0)
    #     # outlier_pub.publish(outlier_ros_cloud)
    #     inlier_object, coefficients, outlier_objects = ransac_line_detection(outlier_objects)

    # if coefficients:
    #     publish_line_marker(inlier_object, coefficients, 3, 1, 1, line_pub3)
    #     print(f"Published line marker line 2: {coefficients}")


def main():
    rospy.init_node('ransac_line_detection_and_visualization_node', anonymous=True)
    line1_pub = rospy.Publisher("/line_one", Marker, queue_size=10)
    line2_pub = rospy.Publisher("/line_two", Marker, queue_size=10)
    # line_pub3 = rospy.Publisher("/visualization_marker3", Marker, queue_size=10)
    outlier_pub = rospy.Publisher('/outlier_cloud', PointCloud2, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)   

    rospy.Subscriber("/point_cloud_topic", PointCloud2, lambda msg: cloud_callback(msg, line1_pub, line2_pub, outlier_pub, tf_buffer))
    
    rospy.spin()

if __name__ == "__main__":
    main()