#include "auto_patrol/ipm.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h> // Added for Marker
#include <geometry_msgs/Point.h>       // Added for Point
#include <tf2_ros/transform_listener.h>    // For TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming points
#include <cmath>
#include <random>
#include <chrono>

// Helper function to time sections
class Timer {
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
public:
    Timer(const std::string& name) : name_(name), start_(std::chrono::high_resolution_clock::now()) {}
    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
        std::cout << name_ << ": " << duration / 1000.0 << " ms" << std::endl;
    }
};

Ipm::Ipm(const std::string& camera_topic) : tf_buffer_(), tf_listener_(tf_buffer_){
    image_sub_ = nh_.subscribe(camera_topic, 1, &Ipm::imageCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 10);
    warped_image_pub_ = nh_.advertise<sensor_msgs::Image>("warped_image_topic", 10);
    line_one_pub_ = nh_.advertise<visualization_msgs::Marker>("/line_one", 10); // New publisher
    ROS_INFO("Ipm initialized with camera topic: %s", camera_topic.c_str());

    K_ << 901.1387623297165, 0, 645.2704047048309,
          0, 902.8926336853814, 366.13035318739895,
          0, 0, 1;
    K_inv_ = K_.inverse();
    T_ << -0.2489, 0.18991, 1.05653;
    R_ = rotationMatrix();

    double scale_x = 500.0 / 1.5;
    double scale_y = -3000.0 / 9.0;
    double x_min = -0.75, y_min = 10.0;
    S_ << scale_x, 0, -x_min * scale_x,
          0, scale_y, -y_min * scale_y,
          0, 0, 1;
}

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

void fitLinesRANSAC(std::vector<cv::Point>& points, std::vector<cv::Vec4i>& lines, 
                    int max_iterations = 500, double distance_threshold = 10.0, 
                    int min_inliers = 300, int max_lines = 2, double min_length = 800.0) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    while (points.size() >= 2 && lines.size() < max_lines) {
        int attempts = 0;
        std::vector<cv::Point> best_inliers;
        double best_a = 0.0, best_b = 0.0, best_c = 0.0;
        double best_length = 0.0;

        while (attempts++ < max_iterations) {
            int idx1 = dis(gen), idx2 = dis(gen);
            while (idx1 == idx2 && points.size() > 1) idx2 = dis(gen);
            cv::Point p1 = points[idx1], p2 = points[idx2];

            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = p2.x * p1.y - p1.x * p2.y;
            double norm = std::sqrt(a * a + b * b);
            if (norm < 1e-6) continue;
            a /= norm; b /= norm; c /= norm;

            std::vector<cv::Point> inliers;
            inliers.reserve(5000); // Pre-allocate
            for (const auto& p : points) {
                double dist = std::abs(a * p.x + b * p.y + c);
                if (dist < distance_threshold) inliers.push_back(p);
            }

            if (inliers.size() >= min_inliers) {
                int min_x = inliers[0].x, max_x = inliers[0].x;
                int min_y = inliers[0].y, max_y = inliers[0].y;
                for (const auto& p : inliers) {
                    min_x = std::min(min_x, p.x);
                    max_x = std::max(max_x, p.x);
                    min_y = std::min(min_y, p.y);
                    max_y = std::max(max_y, p.y);
                }
                double dx = max_x - min_x;
                double dy = max_y - min_y;
                double length = std::sqrt(dx * dx + dy * dy);

                if (length > best_length && length >= min_length) {
                    best_inliers = std::move(inliers);
                    best_a = a; best_b = b; best_c = c;
                    best_length = length;
                    if (best_inliers.size() > points.size() / 2 && best_length > 2000.0) {
                        break; // Early exit
                    }
                }
            }
        }

        if (best_inliers.size() >= min_inliers && best_length >= min_length) {
            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            int min_x = 499, max_x = 0, min_y = 2999, max_y = 0;
            int n = best_inliers.size();
            for (const auto& p : best_inliers) {
                sum_x += p.x; sum_y += p.y; sum_xy += p.x * p.y; sum_xx += p.x * p.x;
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            }
            double mean_x = sum_x / n;
            double mean_y = sum_y / n;
            double slope = (sum_xy - n * mean_x * mean_y) / (sum_xx - n * mean_x * mean_x + 1e-6);
            double intercept = mean_y - slope * mean_x;

            int x1, y1, x2, y2;
            if (std::abs(slope) > 1.0) {
                y1 = min_y; y2 = max_y;
                x1 = static_cast<int>((min_y - intercept) / slope);
                x2 = static_cast<int>((max_y - intercept) / slope);
            } else {
                x1 = min_x; x2 = max_x;
                y1 = static_cast<int>(slope * min_x + intercept);
                y2 = static_cast<int>(slope * max_x + intercept);
            }
            x1 = std::max(0, std::min(499, x1));
            x2 = std::max(0, std::min(499, x2));
            y1 = std::max(0, std::min(2999, y1));
            y2 = std::max(0, std::min(2999, y2));

            lines.push_back(cv::Vec4i(x1, y1, x2, y2));
            ROS_INFO("RANSAC line %zu: inliers=%zu, length=%f", lines.size(), best_inliers.size(), best_length);

            std::vector<cv::Point> remaining;
            remaining.reserve(points.size());
            for (const auto& p : points) {
                double dist = std::abs(best_a * p.x + best_b * p.y + best_c);
                if (dist >= distance_threshold) remaining.push_back(p);
            }
            points = std::move(remaining);
        } else {
            break;
        }
    }
}

std::vector<std::pair<double, double>> Ipm::computeLineCoordinates(const sensor_msgs::ImageConstPtr& msg) {
    auto timer = std::make_unique<Timer>("IPM Processing"); // Timer starts here
    
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

    Eigen::Matrix3d H = S_ * H_world_to_img.inverse() * K_inv_;

    cv::Mat H_cv;
    cv::eigen2cv(H, H_cv);

    cv::Mat warped;
    cv::warpPerspective(cv_ptr->image, warped, H_cv, cv::Size(500, 3000)); // we need to fully understand how it does this scaling
    cv::Mat warpedCopy = warped.clone();

    timer.reset(); // Timer ends here

    // Pre-processing: Thin the line to approximate centerline
    cv::GaussianBlur(warped, warped, cv::Size(5, 5), 0);
    cv::threshold(warped, warped, 128, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(warped, warped, kernel, cv::Point(-1, -1), 5); // Thin aggressively
    
    // RANSAC INTEGRATION 
    // Get centerline points
    std::vector<cv::Point> points;
    cv::findNonZero(warped, points);
    ROS_INFO("Number of centerline points: %zu", points.size());

    // Subsample points for speed (e.g., every 5th point) - seems like slow operation for 400367 points?
    std::vector<cv::Point> sampled_points;
    for (size_t i = 0; i < points.size(); i += 5) {
        sampled_points.push_back(points[i]);
    }
    ROS_INFO("Number of sampled points: %zu", sampled_points.size());

    // Fit lines with RANSAC
    std::vector<cv::Vec4i> lines;
    if (!sampled_points.empty()) {
        fitLinesRANSAC(sampled_points, lines);
    }
    ROS_INFO("Number of lines detected: %zu", lines.size());

    cv::Mat warped_with_lines;

    cv::cvtColor(warped, warped_with_lines, cv::COLOR_GRAY2BGR);

    int index = 0;
    for (const auto& line : lines) {
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);
        // Alternate colors: red for even index, green for odd index
        cv::Scalar color = (index % 2 == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::line(warped_with_lines, pt1, pt2, color, 5);
        index++;
    }

    sensor_msgs::ImagePtr warped_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", warpedCopy).toImageMsg();
    // sensor_msgs::ImagePtr warped_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", warped_with_lines).toImageMsg();
    warped_msg->header = msg->header;
    warped_image_pub_.publish(warped_msg);

    double scale_x = 1.5 / 500.0, scale_y = -9.0 / 3000.0;
    double x_min = -0.75, y_min = 10.0;
    std::vector<std::pair<double, double>> coordinates;
    for (const auto& line : lines) {
        double x1 = line[0] * scale_x + x_min;
        double y1 = line[1] * scale_y + y_min;
        double x2 = line[2] * scale_x + x_min;
        double y2 = line[3] * scale_y + y_min;
        coordinates.emplace_back(x1, y1);
        coordinates.emplace_back(x2, y2);
    }

    // Publish the first line in the map frame
    if (!lines.empty()) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = msg->header.stamp;
        line_marker.ns = "lines";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        // Original points in base_link frame
        geometry_msgs::Point p1_base, p2_base;
        p1_base.x = coordinates[0].first;
        p1_base.y = coordinates[0].second;
        p1_base.z = 0.0;
        p2_base.x = coordinates[1].first;
        p2_base.y = coordinates[1].second;
        p2_base.z = 0.0;

        try {
            // Lookup transform from base_link to map
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));

            // Transform points
            geometry_msgs::Point p1_map, p2_map;
            tf2::doTransform(p1_base, p1_map, transform);
            tf2::doTransform(p2_base, p2_map, transform);

            // Add transformed points to marker
            line_marker.points = {p1_map, p2_map};

            ROS_INFO("Transformed p1: x=%f, y=%f, z=%f", p1_map.x, p1_map.y, p1_map.z);
            ROS_INFO("Transformed p2: x=%f, y=%f, z=%f", p2_map.x, p2_map.y, p2_map.z);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Failed to transform line points: %s", ex.what());
            // Fallback: Publish untransformed points
            line_marker.points = {p1_base, p2_base};
        }

        line_marker.scale.x = 0.05;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        line_marker.lifetime = ros::Duration(0);

        line_one_pub_.publish(line_marker);
    }

    return coordinates;
}

void Ipm::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    auto coordinates = computeLineCoordinates(msg);
    //lets publish the coordinates as markers but
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipm_node");
    Ipm ipm("/cam1/lane_mask");
    ros::spin();
    return 0;
}