#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <chrono>
#include <cmath> 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point_type;
typedef bg::model::segment<point_type> segment_type;

struct EndpointState {
    bool stateOne;
    bool stateTwo;
    double orientation;

    bool operator==(const EndpointState& other) const {
        return stateOne == other.stateOne && stateTwo == other.stateTwo && orientation == other.orientation;
    }
};

// global 
std::vector<std::pair<segment_type, EndpointState>> segments;
std::vector<std::pair<segment_type, EndpointState>> results;

using rtree_type = bgi::rtree<std::pair<segment_type, EndpointState>, bgi::linear<16>>;
rtree_type rtree;

void queryCheck(const point_type& centroid, const segment_type& linestring, const double& orientation)
{
    results.clear();

    // Tolerance for orientation similarity in degrees
    double orientation_tolerance = 2.0; // degrees
    double max_distance = 0.05; // 50mm converted to meters for centroid proximity check
    
    // // Lambda function to convert degrees to radians for the orientation check
    // auto degreesToRadians = [](double degrees) {
    //     return degrees * M_PI / 180.0;
    // };

    // double orientation_tolerance_rad = degreesToRadians(orientation_tolerance);

    // First check: Intersection with similar orientation lines
    rtree.query(bgi::intersects(linestring) && bgi::satisfies([&](const std::pair<segment_type, EndpointState>& v) {
        double orientation_diff = std::abs(v.second.orientation - orientation);
        return orientation_diff <= orientation_tolerance;
    }), std::back_inserter(results));

    if (results.empty()) {
        // Second check: Nearby lines with similar orientation
        rtree.query(bgi::nearest(centroid, 5) && bgi::satisfies([&](const std::pair<segment_type, EndpointState>& v) {
            double orientation_diff = std::abs(v.second.orientation - orientation);
            point_type segment_centroid;
            bg::centroid(v.first, segment_centroid);
            double distance = bg::distance(segment_centroid, centroid);
            return (orientation_diff <= orientation_tolerance) && (distance <= max_distance);
        }), std::back_inserter(results));
    }

    if (results.empty()) {
        // If no matching line found, insert the new line
        rtree.insert(std::make_pair(linestring, EndpointState{false, false, orientation}));
        std::cout << "Line inserted." << std::endl;
    } else {
        std::cout << "Intersecting or nearby similar lines found: " << results.size() << std::endl;
    }

    // 


}

void markerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    // ROS_INFO("Received marker with ID %d", msg->id);
    // process the marker message here
    if (msg->points.size() >= 2) {
        // Extract the points from the Marker message
        geometry_msgs::Point ros_point_1 = msg->points[0];
        geometry_msgs::Point ros_point_2 = msg->points[1];

        // Convert the ROS points to boost::geometry points
        point_type point_1(ros_point_1.x, ros_point_1.y);
        point_type point_2(ros_point_2.x, ros_point_2.y);

        // Process the points (e.g., store them, use them for calculations, etc.)
        // std::cout << "Point 1: (" << point_1.get<0>() << ", " << point_1.get<1>() << ")" << std::endl;
        // std::cout << "Point 2: (" << point_2.get<0>() << ", " << point_2.get<1>() << ")" << std::endl;
        
        // Process points to ensure point 1 x is smaller. Pushing into correct bounds.
        if (point_1.get<0>() > point_2.get<0>() || (point_1.get<0>() == point_2.get<0>() && point_1.get<1>() > point_2.get<1>())) {
            std::swap(point_1, point_2);
        }

        point_type centroid((point_1.get<0>() + point_2.get<0>()) / 2, (point_1.get<1>() + point_2.get<1>()) / 2);

        double length = std::sqrt(
            std::pow(point_2.get<0>() - point_1.get<0>(), 2) +
            std::pow(point_2.get<1>() - point_1.get<1>(), 2)
        );  

        double orientation = std::atan2(point_2.get<1>() - point_1.get<1>(), point_2.get<0>() - point_1.get<0>());

        std::cout << "Point 1: (" << point_1.get<0>() << ", " << point_1.get<1>() << ")" << std::endl;
        std::cout << "Point 2: (" << point_2.get<0>() << ", " << point_2.get<1>() << ")" << std::endl;

        if (orientation < 0) {
            // std::cout << "NEGATIVE ANGLE!: " << orientation * 180 / M_PI  << std::endl;
            orientation += M_PI; // should this be 2pi..     
        }

        // std::cout << "POSITIVE ANGLE!: " << 180 * orientation / M_PI << std::endl;
        orientation = orientation * 180 / M_PI;   

        if (length >= 1.5) {
            // rtree.insert(std::make_pair(segment_type(point_1, point_2), EndpointState{false, false, orientation}));
            queryCheck(centroid, segment_type(point_1, point_2), orientation);
            // mergedetect(); 
            // splitdetect();
            // spotdetection();
        }


    } else {
        ROS_WARN("Received marker with insufficient points.");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "auto_patrol");
    ros::NodeHandle nh;

    ros::Subscriber marker_sub = nh.subscribe("/line_one", 10, markerCallback);
    // ros::Subscriber marker_sub = nh.subscribe("/line_two", 10, markerCallback);

    ros::spin();

    // // Create a vector of segments and their corresponding endpoint states
    // std::vector<std::pair<segment_type, EndpointState>> segments;

    // //SPOT A Diagonal
    // segments.push_back(std::make_pair(segment_type(point_type(2.0, 1.0), point_type(3.5, 2.0)), EndpointState{true, false}));
    // segments.push_back(std::make_pair(segment_type(point_type(3.5, 2.0), point_type(3.5, 4.0)), EndpointState{false, true}));
    // segments.push_back(std::make_pair(segment_type(point_type(2.0, 3.0), point_type(3.5, 4.0)), EndpointState{true, true}));

    // //SPOT B Diagonal
    // segments.push_back(std::make_pair(segment_type(point_type(3.5, 2.0), point_type(5.0, 3.0)), EndpointState{true, true}));
    // segments.push_back(std::make_pair(segment_type(point_type(3.5, 4.0), point_type(5.0, 5.0)), EndpointState{true, true}));

    // // Create an R-tree index for the segments
    // using rtree_type = bgi::rtree<std::pair<segment_type, EndpointState>, bgi::linear<16>>;
    // rtree_type rtree(segments.begin(), segments.end());

    // //   rtree.insert(std::make_pair(segment_type(point_type(6.0, 4.0), point_type(4.0, 5.0)), EndpointState{true, true}));

    // // Query the R-tree for the nearest segment to a point
    // point_type query_point(3.5, 3.0);
    // std::vector<std::pair<segment_type, EndpointState>> results;
    // rtree.query(bgi::nearest(query_point, 5), std::back_inserter(results));

    return 0;
}
