#include "auto_patrol/ipm.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <cmath>

Ipm::Ipm(const std::string& camera_topic) {
    // Initialize ROS
    image_sub_ = nh_.subscribe(camera_topic, 1, &Ipm::imageCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 10);
    warped_image_pub_ = nh_.advertise<sensor_msgs::Image>("warped_image_topic", 10);
    ROS_INFO("Ipm initialized with camera topic: %s", camera_topic.c_str());

    // Initialize constant matrices and vectors
    K_ << 901.1387623297165, 0, 645.2704047048309,
          0, 902.8926336853814, 366.13035318739895,
          0, 0, 1;
    K_inv_ = K_.inverse();
    T_ << -0.2489, 0.18991, 1.05653;
    R_ = rotationMatrix();  // Precompute R once
    
    double scale_x = 500.0 / 1.5;
    double scale_y = 3000.0 / 9.0;
    double x_min = -0.75, y_min = 1.0;
    S_ << scale_x, 0, -x_min * scale_x,
          0, scale_y, -y_min * scale_y,
          0, 0, 1;
}

// Uses XYZ Euler angles - I think
Eigen::Matrix3d Ipm::rotationMatrix() {
    double i = roll_, j = pitch_, k = yaw_;
    double si = std::sin(i), ci = std::cos(i);
    double sj = std::sin(j), cj = std::cos(j);
    double sk = std::sin(k), ck = std::cos(k);
    double cc = ci * ck, cs = ci * sk, sc = si * ck, ss = si * sk;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0, 0) = cj * ck;
    R(0, 1) = sj * sc - cs;
    R(0, 2) = sj * cc + ss;
    R(1, 0) = cj * sk;
    R(1, 1) = sj * ss + cc;
    R(1, 2) = sj * cs - sc;
    R(2, 0) = -sj;
    R(2, 1) = cj * si;
    R(2, 2) = cj * ci;
    return R;
}

std::vector<std::pair<double, double>> Ipm::computeLineCoordinates(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return {};
    }

    Eigen::Matrix3d R_cam2img;
    R_cam2img << 0, -1, 0,
                 0, 0, -1,
                 1, 0, 0;

    Eigen::Matrix3d R_w2c = R_.transpose();
    Eigen::Vector3d T_w2c = -(R_w2c * T_);

    Eigen::Matrix3d R_w2i = R_cam2img * R_w2c;
    Eigen::Vector3d T_w2i = R_cam2img * T_w2c;

    Eigen::Matrix3d H_world_to_img;
    H_world_to_img << R_w2i(0, 0), R_w2i(0, 1), T_w2i(0),
                      R_w2i(1, 0), R_w2i(1, 1), T_w2i(1),
                      R_w2i(2, 0), R_w2i(2, 1), T_w2i(2);

    // Eigen::Matrix3d H_test = H_world_to_img.inverse() * K_inv_;
    // ROS_INFO_STREAM("Homography matrix:\n" << H_test);

    Eigen::Matrix3d H = S_ * H_world_to_img.inverse() * K_inv_;

    cv::Mat H_cv;
    cv::eigen2cv(H, H_cv);  // Back to using eigen2cv with proper header

    cv::Mat warped;
    cv::warpPerspective(cv_ptr->image, warped, H_cv, cv::Size(500, 3000));

    cv::Mat edges;
    cv::Canny(warped, edges, 100, 200);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 150, 500, 100); //detecting 1 to 3 lines with these parameters

    ROS_INFO("Number of lines detected by Hough transform: %zu", lines.size()); 

    // Convert warped to BGR for colored lines
    cv::Mat warped_with_lines;
    cv::cvtColor(warped, warped_with_lines, cv::COLOR_GRAY2BGR);

    // Draw Hough lines on the image
    for (const auto& line : lines) {
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);
        cv::line(warped_with_lines, pt1, pt2, cv::Scalar(0, 0, 255), 3); // Red lines, thickness 2
    }

    // Publish the warped image with lines
    sensor_msgs::ImagePtr warped_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", warped_with_lines).toImageMsg();
    warped_msg->header = msg->header;
    warped_image_pub_.publish(warped_msg);

    double scale_x = 1.5 / 500.0, scale_y = 9.0 / 3000.0;
    double x_min = -0.75, y_min = 1.0;
    std::vector<std::pair<double, double>> coordinates;
    for (const auto& line : lines) {
        double x1 = line[0] * scale_x + x_min;
        double y1 = line[1] * scale_y + y_min;
        double x2 = line[2] * scale_x + x_min;
        double y2 = line[3] * scale_y + y_min;
        coordinates.emplace_back(x1, y1);
        coordinates.emplace_back(x2, y2);
    }
    return coordinates;
}

void Ipm::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // ROS_INFO_STREAM("Rotation matrix (precomputed):\n" << R_);
    auto coordinates = computeLineCoordinates(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipm_node");
    Ipm ipm("/cam1/lane_mask");
    ros::spin();
    return 0;
}
    // try {
    //     auto start_time = ros::Time::now().toSec();

    //     // Convert ROS image to OpenCV
    //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    //     cv::Mat img = cv_ptr->image;

    // } catch (cv_bridge::Exception& e) {
    //     ROS_ERROR("CV Bridge error: %s", e.what());  
    // }