#ifndef IPM_H
#define IPM_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

};

#endif // IPM_H