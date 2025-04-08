#ifndef IPM_H
#define IPM_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>     // For Marker
#include <geometry_msgs/Point.h>           // For Point
#include <tf2_ros/transform_listener.h>     // For TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming points
#include <sensor_msgs/Image.h>
#include <vector>

class Ipm {
public:
    Ipm(const std::string& camera_topic);
    Eigen::Matrix3d rotationMatrix();
    std::vector<std::pair<double, double>> computeLineCoordinates(const sensor_msgs::ImageConstPtr& msg);
    

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    // Camera extrinsic parameters
    double roll_ = 0.0;
    double pitch_ = 0.108;
    double yaw_ = 1.57;

    // Precomputed matrices and vectors
    Eigen::Matrix3d K_;         // Intrinsic matrix
    Eigen::Matrix3d K_inv_;     // Inverse of intrinsic matrix
    Eigen::Vector3d T_;         // Translation vector
    Eigen::Matrix3d R_;         // Rotation matrix (precomputed)
    Eigen::Matrix3d S_;         // Scaling matrix

    // ROS members
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher warped_image_pub_;
    ros::Publisher line_one_pub_;

    tf2_ros::Buffer tf_buffer_;              // TF2 buffer
    tf2_ros::TransformListener tf_listener_; // TF2 listener

};

#endif // IPM_H