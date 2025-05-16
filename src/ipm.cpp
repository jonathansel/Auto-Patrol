#include "auto_patrol/ipm.h"
#include <cmath>
#include <random>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>    // For TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming points
#include <visualization_msgs/Marker.h> // Added for Marker
#include <geometry_msgs/Point.h>       // Added for Point
#include "auto_patrol/LineArray.h" // Add custom message

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

Ipm::Ipm() : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
    // Load parameters from YAML
    loadParameters();

    // Initialize ROS subscribers and publishers
    image_sub_ = nh_.subscribe(nh_.param<std::string>("camera/topic", "/cam1/lane_mask"), 1, &Ipm::imageCallback, this);
    warped_image_pub_ = nh_.advertise<sensor_msgs::Image>("warped_image_topic", 10);
    // line_one_pub_ = nh_.advertise<visualization_msgs::Marker>("/line_one", 10);
    line_one_pub_ = nh_.advertise<auto_patrol::LineArray>("/line_array", 10);


    ROS_INFO("IPM initialized!");
}

void Ipm::loadParameters() {
    // Load intrinsic parameters
    ROS_INFO("Loading Parameters");

    double fx, fy, cx, cy;
    nh_.param("intrinsics/fx", fx, 901.1387623297165);
    nh_.param("intrinsics/fy", fy, 902.8926336853814);
    nh_.param("intrinsics/cx", cx, 645.2704047048309);
    nh_.param("intrinsics/cy", cy, 366.13035318739895);
    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;
    K_inv_ = K_.inverse();

    // Load extrinsic parameters
    nh_.param("extrinsics/roll", roll_, 0.0);
    nh_.param("extrinsics/pitch", pitch_, 0.108);
    nh_.param("extrinsics/yaw", yaw_, 1.57);
    double tx, ty, tz;
    nh_.param("extrinsics/tx", tx, -0.2489);
    nh_.param("extrinsics/ty", ty, 0.18991);
    nh_.param("extrinsics/tz", tz, 1.05653);
    T_ << tx, ty, tz;
    R_ = rotationMatrix();

    // Load scaling parameters
    double scale_x, scale_y, x_min, y_min;
    nh_.param("scaling/scale_x", scale_x, 500.0 / 1.5);
    nh_.param("scaling/scale_y", scale_y, -3000.0 / 9.0);
    nh_.param("scaling/x_min", x_min, -0.75);
    nh_.param("scaling/y_min", y_min, 10.0);
    S_ << scale_x, 0, -x_min * scale_x,
          0, scale_y, -y_min * scale_y,
          0, 0, 1;

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

    cv::eigen2cv(H, H_cv);

    ROS_INFO("Parameters loaded from parameter server");
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

struct LineSegment {
    cv::Vec4i line;
    cv::Point2f start;
    cv::Point2f end;
    double length;
    double angle;
    cv::Point2f centroid; 
};

// Function to compute direction vector from p1 to p2 /// could use seg
std::pair<double, double> directionVector(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::make_pair(p2.x - p1.x, p2.y - p1.y);
}

double magnitude(const std::pair<double, double>& v) {
    return std::sqrt(v.first * v.first + v.second * v.second);
}

double crossProduct(const std::pair<double, double>& v1, const std::pair<double, double>& v2) {
    return v1.first * v2.second - v1.second * v2.first;
}

double dotProduct(const std::pair<double, double>& v1, const std::pair<double, double>& v2) {
    return v1.first * v2.first + v1.second * v2.second;
}

// Function to check angular proximity between two line segments
// seg 1 is P1P2 seg 2 is Q1Q2 (line segment candidate)
bool angularProximityCheck(const LineSegment& seg1, const LineSegment& seg2, const double& tau_theta) {
    auto direction1 = directionVector(seg1.start, seg1.end);
    auto direction2 = directionVector(seg2.start, seg2.end);

    double cross_prod = crossProduct(direction1, direction2);
    double mag1 = magnitude(direction1);
    double mag2 = magnitude(direction2);

    double angular_proximity = std::abs(cross_prod) / (mag1 * mag2);

    return angular_proximity < std::sin(tau_theta);
}

bool verticalProximityCheck(const LineSegment& seg1, const LineSegment& seg2, const double& vert_thresh) {
    //just calc centroid here?
    auto direction1 = directionVector(seg1.start, seg2.centroid); //P1QM, note centroid is of the query segment, in rtree that would be the one that triggered the search, maybe have to swap?
    auto direction2 = directionVector(seg1.start, seg1.end); //P1P2

    double cross_prod = crossProduct(direction1, direction2);
    double mag2 = magnitude(direction2);
    double vertical_proximity = std::abs(cross_prod) / (mag2);
    // std::cout << "VERTICAL PROXIMITY: " << vertical_proximity << std::endl;
 
    return vertical_proximity <= vert_thresh;
}

bool longitudeProximityCheck(const LineSegment& seg1, const LineSegment& seg2, const double& spatial_thresh){
    // Longitudinal proximity
    // seg 1 expected to be segments[i], i.e. main line of current cluster
    auto seg1_centroid = cv::Point2f(seg1.centroid.x, seg1.centroid.y);        
    auto seg2_centroid = cv::Point2f(seg2.centroid.x, seg2.centroid.y);       

    auto direction1 = directionVector(seg1.start, seg1.end); // P1P2
    auto direction2 = directionVector(seg1_centroid, seg2_centroid); // Pm to Qm

    double dot_prod = dotProduct(direction1, direction2);
    double mag1 = magnitude(direction1); 
    double longitudinal_prox = std::abs(dot_prod) / mag1;

    return longitudinal_prox < spatial_thresh; // study output, margin used for parkingdetect, maybe diff here
}

//Hough Transform
std::vector<cv::Vec4i> lineDetection(cv::Mat & image) {
    auto timer = std::make_unique<Timer>("Line Detection"); // Timer starts here

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        ROS_INFO("No contours detected, returning empty lines");
        return std::vector<cv::Vec4i>();
    }

    // Create a new black image (same size as binary_mask)
    cv::Mat filtered_mask = cv::Mat::zeros(image.size(), CV_8UC1); // Black (0) background

    // Process each contour
    std::vector<std::vector<cv::Point>> valid_contours; // Store contours with compactness >= 10
    for (size_t i = 0; i < contours.size(); ++i) {
        double length = arcLength(contours[i], true);
        double area = contourArea(contours[i]);

        // Skip contours with zero or very small area to avoid division by zero
        if (area < 1e-5) {
            std::cout << "Contour " << i << ": Skipped (area too small)" << std::endl;
            continue;
        }

        // Calculate compactness
        double compactness = (length * length) / (4 * CV_PI * area);
        std::cout << "Contour " << i << ": Compactness=" << compactness << std::endl;

        // Keep contours with compactness >= 10
        if (compactness >= 10) {
            valid_contours.push_back(contours[i]);
        } else {
            std::cout << "Contour " << i << ": Removed (compactness < 10)" << std::endl;
        }
    }

    // Early exit if no valid contours
    if (valid_contours.empty()) {
        ROS_INFO("No valid contours (compactness >= 10), returning empty lines");
        return std::vector<cv::Vec4i>();
    }

    cv::drawContours(filtered_mask, valid_contours, -1, cv::Scalar(255), cv::FILLED);

    std::vector<cv::Vec4i> lines;
    cv::Mat resized_image;

    float fx = 0.50;  // Scale factor for width
    float fy = 0.50;  // Scale factor for height
    cv::resize(filtered_mask, resized_image, cv::Size(), fx, fy, cv::INTER_LINEAR);  // INTER_LINEAR is a common interpolation method
    cv::erode(resized_image, resized_image, cv::Mat(), cv::Point(-1, -1), 3); // Thin aggressively

    cv::HoughLinesP(resized_image, lines, 1, CV_PI/180, 100, 75, 25);

    // Early exit if no lines detected
    if (lines.empty()) {
        ROS_INFO("No lines detected by Hough, returning empty lines");
        return std::vector<cv::Vec4i>();
    }

    double vert_thresh = 150; // 150 pixels in assumption this encapsulates line width with some tolerance. 500px width - modify as necessary
    double tau_theta = 0.0872665; // 3 deg: 0.0523599 5 deg: 0.0872665 10 deg: 0.174533
    double tau_s = 2000; //for spatial non-overlapping scenarios

    // Rescale lines back to original image size 
    for (auto& line : lines) {
        line[0] = static_cast<int>(line[0] / fx);
        line[1] = static_cast<int>(line[1] / fy);
        line[2] = static_cast<int>(line[2] / fx);
        line[3] = static_cast<int>(line[3] / fy);
    }

    // Create a vector of LineSegment structs to store line properties
    std::vector<LineSegment> segments; // holds all line detections from HoughLinesP with struct per line
    for (auto& line: lines) {
        LineSegment seg;
        seg.line = line;
        seg.start = cv::Point2f(line[0], line[1]);
        seg.end = cv::Point2f(line[2], line[3]);        
        seg.length = std::sqrt(std::pow(line[2] - line[0], 2) + std::pow(line[3] - line[1], 2)); //x2 - x1, y2-y1
        seg.angle = std::atan2(line[3] - line[1], line[2] - line[0]);
        seg.centroid = cv::Point2f((line[0] + line[2]) / 2.0, (line[1] + line[3]) / 2.0);
        segments.push_back(seg);
    }

    std::vector<std::vector<LineSegment>> clusters;
    std::vector<bool> processed(segments.size(), false);

    // DBSCAN-based Line Clustering
    // I essentially want to remove a seg from segments once its clustered. so the first line should be a cluster.
    for (size_t i = 0; i < segments.size(); ++i) {
        if (processed[i]) continue;
        std::vector<LineSegment> cluster;
        cluster.push_back(segments[i]);
        processed[i] = true;
        
        for (size_t j = 0; j < segments.size(); ++j){
            if (processed[j]) continue;
            
            // check segment send order and thresholds
            auto angular_ok = angularProximityCheck(segments[i], segments[j], tau_theta);
            auto vertical_ok = verticalProximityCheck(segments[i], segments[j], vert_thresh);
            auto long_ok = longitudeProximityCheck(segments[i], segments[j], tau_s);

            if (angular_ok && vertical_ok && long_ok) {
                cluster.push_back(segments[j]);
                processed[j] = true;
            }
            
            // std::string angular_str = angular_ok ? "TRUE" : "FALSE";
            // std::string vertical_str = vertical_ok ? "TRUE" : "FALSE";
            // std::string spatial_str = spatial_ok ? "TRUE" : "FALSE";
            // ROS_INFO("Proximity checks: angular=%s, vertical=%s, spatial=%s",
            //             angular_str.c_str(), vertical_str.c_str(), spatial_str.c_str());
        }

        clusters.push_back(cluster); // Store the cluster
    }

    std::vector<cv::Vec4i> representative_lines;
    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;
        LineSegment longest_line = cluster[0]; // Initialize with first segment
        for (size_t j = 1; j < cluster.size(); ++j) {
            if (cluster[j].length > longest_line.length) {
                longest_line = cluster[j];
            }
        }
        if (longest_line.length >= 50) { // Length filter
            representative_lines.push_back(longest_line.line);
        }
    }

    // For outputting the clusters 
    // std::vector<std::vector<cv::Vec4i>> cluster_lines;
    // for (const auto& cluster : clusters) {
    //     if (cluster.empty()) continue;
    //     std::vector<cv::Vec4i> cluster_line_vec;
    //     for (const auto& line : cluster){
    //         cluster_line_vec.push_back(line.line);
    //     }
    //     cluster_lines.push_back(cluster_line_vec);
    // }

    timer.reset(); // Timer ends here

    return representative_lines; 
}

// std::vector<std::pair<double, double>>
void Ipm::computeLineCoordinates(const sensor_msgs::ImageConstPtr& msg) {
    auto timer = std::make_unique<Timer>("IPM Processing"); // Timer starts here

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        // return {};
    }

    cv::Mat warped;
    cv::warpPerspective(cv_ptr->image, warped, H_cv, cv::Size(500, 3000)); // we need to fully understand how it does this scaling
    // cv::Mat warpedCopy = warped.clone();

    // Pre-processing: Thin the line to approximate centerline
    // cv::GaussianBlur(warped, warped, cv::Size(5, 5), 0);


    cv::threshold(warped, warped, 128, 255, cv::THRESH_BINARY);
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); //whats the difference between this and cv::Mat()

    // Get centerline points
    // std::vector<cv::Point> points;
    // cv::findNonZero(warped, points); //CHECK: Profile, isn't used right now anyways
    // ROS_INFO("Number of centerline points: %zu", points.size());

    // Subsample points for speed (e.g., every 5th point) - seems like slow operation for 400367 points?
    // std::vector<cv::Point> sampled_points;
    // for (size_t i = 0; i < points.size(); i += 5) {
    //     sampled_points.push_back(points[i]);
    // }
    // ROS_INFO("Number of sampled points: %zu", sampled_points.size());

    // Fit lines with Hough
    // std::vector<cv::Vec4i> lines;
    // if (!sampled_points.empty()) {
    //     auto lines = lineDetection(warped);
    // }

    auto rep_lines = lineDetection(warped);
    
    if (rep_lines.empty()) {
        ROS_INFO("No representative lines detected, skipping visualization and publishing");
        return;
    }
    // ROS_INFO("Number of clusters detected: %zu", clusters.size());

    // size_t total_lines = 0;
    // for (const auto& cluster : clusters) {
    //     total_lines += cluster.size();
    // }
    // ROS_INFO("Number of lines detected: %zu", total_lines);

    // ROS_INFO("Number of lines detected: %zu", lines.size());

    cv::Mat warped_with_lines;

    cv::cvtColor(warped, warped_with_lines, cv::COLOR_GRAY2BGR);

    // // for representative_lines
    int index = 0;
    for (const auto& line : rep_lines) {
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);
        // Alternate colors: red for even index, green for odd index
        cv::Scalar color = (index % 2 == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::line(warped_with_lines, pt1, pt2, color, 5);
        index++;
    }

    // Initialize random number generator
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<int> dist(0, 255);

    // for (const auto& cluster : clusters) {
    //     // Generate random color for this cluster
    //     cv::Scalar color(dist(gen), dist(gen), dist(gen)); // Random BGR

    //     for (const auto& line : cluster) {
    //         cv::Point pt1(line[0], line[1]);
    //         cv::Point pt2(line[2], line[3]);
    //         cv::line(warped_with_lines, pt1, pt2, color, 1);
    //     }
    // }

    // sensor_msgs::ImagePtr warped_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", warpedCopy).toImageMsg();

    // downscale if slow and remove if not debugging
    sensor_msgs::ImagePtr warped_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", warped_with_lines).toImageMsg();
    warped_msg->header = msg->header;
    warped_image_pub_.publish(warped_msg);

    auto timer2 = std::make_unique<Timer>("Point Processing"); // Timer starts here

    // Publish LineArray
    auto_patrol::LineArray line_array;
    line_array.header.stamp = msg->header.stamp;
    line_array.header.frame_id = "map";

    // Pixel-to-meter scaling (500 pixels = 1.5 meters, 3000 pixels = 9 meters)
    const double scale_x = 1.5 / 500.0; // meters per pixel
    const double scale_y = -9.0 / 3000.0; // meters per pixel (negative for BEV orientation)
    const double x_min = -0.75; // From loadParameters
    const double y_min = 10.0;

// Cache transform once
    geometry_msgs::TransformStamped transform;
    try {
        auto timer3 = std::make_unique<Timer>("Transform Lookup");
        transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return;
    }

    for (const auto& line : rep_lines) {
        // Convert pixel coordinates to base_link frame (meters)
        geometry_msgs::PointStamped pt1, pt2;
        pt1.header.frame_id = "base_link";
        pt1.header.stamp = msg->header.stamp; // Use image timestamp
        pt1.point.x = line[0] * scale_x + x_min;
        pt1.point.y = line[1] * scale_y + y_min;
        pt1.point.z = 0.0;

        pt2.header.frame_id = "base_link";
        pt2.header.stamp = msg->header.stamp;
        pt2.point.x = line[2] * scale_x + x_min;
        pt2.point.y = line[3] * scale_y + y_min;
        pt2.point.z = 0.0;

        // Apply cached transform
        geometry_msgs::PointStamped pt1_map, pt2_map;
        auto timer4 = std::make_unique<Timer>("Transform Apply");
        tf2::doTransform(pt1, pt1_map, transform);
        tf2::doTransform(pt2, pt2_map, transform);

        // Create Line message
        auto_patrol::Line line_msg;
        line_msg.start = pt1_map.point;
        line_msg.end = pt2_map.point;

        line_array.lines.push_back(line_msg);
    }

    line_one_pub_.publish(line_array);
    timer2.reset();
    timer.reset();

    // double scale_x = 1.5 / 500.0, scale_y = -9.0 / 3000.0;
    // double x_min = -0.75, y_min = 10.0;
    // std::vector<std::pair<double, double>> coordinates;
    // for (const auto& line : lines) {
    //     double x1 = line[0] * scale_x + x_min;
    //     double y1 = line[1] * scale_y + y_min;
    //     double x2 = line[2] * scale_x + x_min;
    //     double y2 = line[3] * scale_y + y_min;
    //     coordinates.emplace_back(x1, y1);
    //     coordinates.emplace_back(x2, y2);
    // }

    // // Publish the first line in the map frame
    // if (!lines.empty()) {
    //     visualization_msgs::Marker line_marker;
    //     line_marker.header.frame_id = "map";
    //     line_marker.header.stamp = msg->header.stamp;
    //     line_marker.ns = "lines";
    //     line_marker.id = 0;
    //     line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    //     line_marker.action = visualization_msgs::Marker::ADD;

    //     // Original points in base_link frame
    //     geometry_msgs::Point p1_base, p2_base;
    //     p1_base.x = coordinates[0].first;
    //     p1_base.y = coordinates[0].second;
    //     p1_base.z = 0.0;
    //     p2_base.x = coordinates[1].first;
    //     p2_base.y = coordinates[1].second;
    //     p2_base.z = 0.0;

    //     try {
    //         Timer t("Transforming");
    //         // Lookup transform from base_link to map
    //         geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));

    //         // Transform points
    //         geometry_msgs::Point p1_map, p2_map;
    //         tf2::doTransform(p1_base, p1_map, transform);
    //         tf2::doTransform(p2_base, p2_map, transform);

    //         // Add transformed points to marker
    //         line_marker.points = {p1_map, p2_map};

    //         ROS_INFO("Transformed p1: x=%f, y=%f, z=%f", p1_map.x, p1_map.y, p1_map.z);
    //         ROS_INFO("Transformed p2: x=%f, y=%f, z=%f", p2_map.x, p2_map.y, p2_map.z);
    //     } catch (tf2::TransformException& ex) {
    //         ROS_WARN("Failed to transform line points: %s", ex.what());
    //         // Fallback: Publish untransformed points
    //         line_marker.points = {p1_base, p2_base};
    //     }

    //     line_marker.scale.x = 0.05;
    //     line_marker.color.r = 1.0;
    //     line_marker.color.g = 0.0;
    //     line_marker.color.b = 0.0;
    //     line_marker.color.a = 1.0;
    //     line_marker.lifetime = ros::Duration(0);

    //     line_one_pub_.publish(line_marker);
    // }

    // return coordinates;
}

void Ipm::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    computeLineCoordinates(msg); 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipm_node");
    Ipm ipm;
    ros::spin();
    return 0;
}