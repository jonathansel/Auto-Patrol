#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <Eigen/Dense>

class InverseProjector {
public:
    InverseProjector(const std::string& camera_topic) {
        image_sub_ = nh_.subscribe(camera_topic, 1, &InverseProjector::imageCallback, this);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

        // Extrinsic orientation of camera
        pitch_ = 0.108;
        roll_ = 0.0;
        yaw_ = 1.57;

        // Extrinsic position of camera [m]
        x_ = -0.2489;
        y_ = 0.18991;
        z_ = 1.05653;

        // Intrinsic parameters of camera
        fx_ = 901.1387623297165;
        fy_ = 902.8926336853814;
        u0_ = 645.2704047048309;
        v0_ = 366.13035318739895;

        K_ << fx_, 0, u0_,
              0, fy_, v0_,
              0,  0,  1;

        R_c2w_ = computeRotationMatrix();
        T_c2w_ << x_, y_, z_;

        R_w2c_ = R_c2w_.transpose();
        T_w2c_ = -R_w2c_ * T_c2w_;

        R_cam2img_ << 0, -1, 0,
                      0,  0, -1,
                      1,  0,  0;

        C_ = K_ * R_cam2img_;
        P_ = C_ * R_w2c_;
        t_ = C_ * T_w2c_;

        arr_ << R_cam2img_ * R_w2c_, R_cam2img_ * T_w2c_;
        arr_inv_ = arr_.inverse();
        K_inv_ = K_.inverse();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            auto start_time = ros::Time::now().toSec();

            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            cv::Mat img = cv_ptr->image;

            // Find indices of pixels with value 255
            std::vector<cv::Point> points_of_interest;
            for (int y = 0; y < img.rows; ++y) {
                for (int x = 0; x < img.cols; ++x) {
                    if (img.at<uint8_t>(y, x) == 255) {
                        points_of_interest.emplace_back(x, y);
                    }
                }
            }

            // Inverse projection to world coordinates
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            for (const auto& point : points_of_interest) {
                Eigen::Vector3d img_coord(point.x, point.y, 1);
                Eigen::Vector3d world_coord = arr_inv_ * K_inv_ * img_coord;

                // Normalize coordinates
                world_coord[0] /= world_coord[2] + 1e-24;
                world_coord[1] /= world_coord[2] + 1e-24;
                world_coord[2] = 0;

                // Apply bounding box limits
                if (world_coord[0] >= -0.75 && world_coord[0] <= 0.75 &&
                    world_coord[1] >= 1.0 && world_coord[1] <= 10.0) {
                    pcl::PointXYZRGB pcl_point;
                    pcl_point.x = world_coord[0];
                    pcl_point.y = world_coord[1];
                    pcl_point.z = world_coord[2];
                    pcl_point.r = 255;
                    pcl_point.g = 255;
                    pcl_point.b = 255;
                    cloud.push_back(pcl_point);
                }
            }

            // Convert to ROS message and publish
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(cloud, cloud_msg);
            cloud_msg.header.frame_id = "base_link";
            cloud_msg.header.stamp = msg->header.stamp;
            pointcloud_pub_.publish(cloud_msg);

            auto end_time = ros::Time::now().toSec();
            ROS_INFO("IPM completed in %.6f seconds", end_time - start_time);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("CV Bridge error: %s", e.what());
        }
    }

private:
    Eigen::Matrix3d computeRotationMatrix() {
        double si = sin(roll_), sj = sin(pitch_), sk = sin(yaw_);
        double ci = cos(roll_), cj = cos(pitch_), ck = cos(yaw_);

        Eigen::Matrix3d R;
        R << cj * ck, sj * sk - cs(ci, ck), sj * ck + ss(ci, sk),
             cj * sk, sj * ss(ci, ck) + cc(ci, sk), sj * cs(ci, ck) - sc(ci, sk),
             -sj,     cj * si,                    cj * ci;
        return R;
    }

    double cc(double ci, double sk) { return ci * sk; }
    double ss(double ci, double ck) { return ci * ck; }
    double sc(double si, double sk) { return si * sk; }
    double cs(double si, double ck) { return si * ck; }

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher pointcloud_pub_;

    double pitch_, roll_, yaw_;
    double x_, y_, z_;
    double fx_, fy_, u0_, v0_;

    Eigen::Matrix3d K_, K_inv_;
    Eigen::Matrix<double, 3, 4> C_, P_, arr_;
    Eigen::Matrix<double, 3, 4> arr_inv_;
    Eigen::Vector3d T_c2w_, T_w2c_;
    Eigen::Matrix3d R_c2w_, R_w2c_, R_cam2img_;
    Eigen::Vector3d t_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverse_perspective_mapping");
    std::string camera_topic = "/cam1/lane_mask";
    InverseProjector projector(camera_topic);
    ros::spin();
    return 0;
}
