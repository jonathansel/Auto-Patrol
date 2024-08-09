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

// global 
// std::vector<segment_type> segments; //delete?
std::vector<segment_type> results;

using rtree_type = bgi::rtree<segment_type, bgi::linear<16>>;
rtree_type rtree;
ros::Publisher marker_pub;

// Function to compute direction vector from p1 to p2
std::pair<double, double> directionVector(const point_type& p1, const point_type& p2) {
    return std::make_pair(p2.get<0>() - p1.get<0>(), p2.get<1>() - p1.get<1>());
}

double magnitude(const std::pair<double, double>& v) {
    return std::sqrt(v.first * v.first + v.second * v.second);
}

double dotProduct(const std::pair<double, double>& v1, const std::pair<double, double>& v2) {
    return v1.first * v2.first + v1.second * v2.second;
}

double crossProduct(const std::pair<double, double>& v1, const std::pair<double, double>& v2) {
    return v1.first * v2.second - v1.second * v2.first;
}

double euclideanDistance(const point_type& p1, const point_type& p2) {
    return bg::distance(p1, p2);
}

// Function to compute the dot product of two vectors
double dotProduct(const point_type& v1, const point_type& v2) {
    return v1.get<0>() * v2.get<0>() + v1.get<1>() * v2.get<1>();
}

// Function to check angular proximity between two line segments
// seg 1 is P1P2 seg 2 is Q1Q2 (line segment candidate)
bool angularProximityCheck(const segment_type& seg1, const segment_type& seg2, const double& tau_theta) {
    auto direction1 = directionVector(seg1.first, seg1.second);
    auto direction2 = directionVector(seg2.first, seg2.second);

    double cross_prod = crossProduct(direction1, direction2);
    double mag1 = magnitude(direction1);
    double mag2 = magnitude(direction2);

    double angular_proximity = std::abs(cross_prod) / (mag1 * mag2);
    // std::cout << "ANGULAR PROXIMITY: " << angular_proximity << std::endl;

    if (angular_proximity > std::sin(tau_theta)){
        std::cout << "angular proximity NG seg: " << bg::wkt(seg1) << ", query seg: " << bg::wkt(seg2) << "prox: " << angular_proximity << " sin: " << std::sin(tau_theta) << std::endl;
    }

    if (angular_proximity < std::sin(tau_theta)){
        std::cout << "angular proximity OK seg: " << bg::wkt(seg1) << ", query seg: " << bg::wkt(seg2) << "prox: " << angular_proximity << " sin: " << std::sin(tau_theta) << std::endl;
    }

    return angular_proximity < std::sin(tau_theta);
}

bool verticalProximityCheck(const segment_type& seg1, const point_type& centroid, const double& vert_thresh) {
    //just calc centroid here?
    auto direction1 = directionVector(seg1.first, centroid); //P1QM
    auto direction2 = directionVector(seg1.first, seg1.second); //P1P2

    double cross_prod = crossProduct(direction1, direction2);
    double mag2 = magnitude(direction2);
    double vertical_proximity = std::abs(cross_prod) / (mag2);
    // std::cout << "VERTICAL PROXIMITY: " << vertical_proximity << std::endl;

    if (vertical_proximity >= vert_thresh){
        std::cout << "vertical proximity NG seg: " << bg::wkt(seg1) << ", centroid: " << bg::wkt(centroid) << "prox: " << vertical_proximity << std::endl;
    }

    return vertical_proximity <= vert_thresh;
}

// seg1 = result (P1P2) seg2 = query (Q1Q2)
void spatialRelation(const segment_type& seg1, const segment_type& seg2, const double& tau_s) {
    // Calculate distances between each pair of endpoints
    double d1 = euclideanDistance(seg1.first, seg2.first);
    double d2 = euclideanDistance(seg1.first, seg2.second);
    double d3 = euclideanDistance(seg1.second, seg2.first);
    double d4 = euclideanDistance(seg1.second, seg2.second);
    double d5 = euclideanDistance(seg1.first, seg1.second); // length of the first segment
    double d6 = euclideanDistance(seg2.first, seg2.second); // length of the second segment

    // we need to find the longest segment. If it is anything beyond d5 (rtree seg) then perhaps we need to update? 
    // do we check every case? 

    double max_distance = d1;
    segment_type longest_segment(seg1.first, seg2.first);

    if (d2 > max_distance) {
        max_distance = d2;
        longest_segment = segment_type(seg1.first, seg2.second);
    }
    if (d3 > max_distance) {
        max_distance = d3;
        longest_segment = segment_type(seg1.second, seg2.first);
    }
    if (d4 > max_distance) {
        max_distance = d4;
        longest_segment = segment_type(seg1.second, seg2.second);
    }
    if (d5 > max_distance) {
        max_distance = d5;
        longest_segment = segment_type(seg1.first, seg1.second);
    }
    if (d6 > max_distance) {
        max_distance = d6;
        longest_segment = segment_type(seg2.first, seg2.second);
    }

    // std::cout << "max distance: " << max_distance << std::endl;

    auto direction1 = directionVector(seg1.first, seg1.second);//p1p2
    auto direction2 = directionVector(seg1.first, seg2.first); //p1q1
    auto direction3 = directionVector(seg1.first, seg2.second); //p1q2
    double mag1 = magnitude(direction1); //mag(p1p2)
    double margin = tau_s * mag1; // 0 <= tau_s <= 1

    //projection q1'
    double q1_proj = dotProduct(direction2, direction1) / (mag1);
    double q2_proj = dotProduct(direction3, direction1) / (mag1);

    double q1q2_mag = std::abs(q1_proj - q2_proj);
    // std::cout << "q1q2 mag: " << q1q2_mag << std::endl;
    double merge_criteria = mag1 + q1q2_mag; 
    
    if (max_distance == mag1) {
        std::cout << "segment contained" << std::endl;
        return;
    }

    if (max_distance - merge_criteria <= margin) {
        std::cout << "segment updated" << std::endl;
        rtree.remove(seg1);
        rtree.insert(longest_segment);
    } else { //need to debug this, I'm, thinking if it's non-overlapping and too far away then you need to insert it. 
        std::cout << "NEW INSERT - SPATIAL" << std::endl;
        rtree.insert(seg2);
    }
}

void visualizeSegments(ros::Publisher& marker_pub) {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map"; // Adjust the frame as necessary
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    
    // Set the scale of the line (line width)
    line_list.scale.x = 0.1;

    // Set the color (red, green, blue, alpha)
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;

    for (const auto& seg : rtree) {
        geometry_msgs::Point p1, p2;
        p1.x = bg::get<0, 0>(seg);
        p1.y = bg::get<0, 1>(seg);
        p1.z = 0; // Assuming a 2D plane, set z to 0 or any constant value

        p2.x = bg::get<1, 0>(seg);
        p2.y = bg::get<1, 1>(seg);
        p2.z = 0;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }

    marker_pub.publish(line_list);
}

void queryCheck(const segment_type& query) {
    results.clear();
    
    // Define the maximum distance for nearest neighbor search in meters - tune
    double vert_thresh = 1; // 0.2 meters or 20cm
    double tau_theta = 0.0872665; // 5 degrees in rad 3: 0.0523599
    double tau_s = 0.8; //for spatial non-overlapping scenarios

    // point_type centroid;
    // bg::centroid(query, centroid);
    point_type centroid((query.first.get<0>() + query.second.get<0>()) / 2.0, 
                    (query.first.get<1>() + query.second.get<1>()) / 2.0);

    rtree.query(bgi::nearest(centroid, 5), std::back_inserter(results)); //change to linestring

    // Remove segments that do not pass the angular proximity check
    results.erase(
        std::remove_if(results.begin(), results.end(), 
            [&query, tau_theta](const segment_type& result) {
                return !angularProximityCheck(result, query, tau_theta);
            }), 
        results.end()
    );

    // Loop through the remaining segments that passed the angular proximity check
    // for (const auto& segment : results) {
    //     // const auto& segment = result.first;
    //     std::cout << "Segment passes angular proximity check: ("
    //               << bg::get<0, 0>(segment) << ", " << bg::get<0, 1>(segment) << ") to ("
    //               << bg::get<1, 0>(segment) << ", " << bg::get<1, 1>(segment) << ")\n";
    // }

    if (results.empty()) {
        std::cout << "SEGMENT INSERTED - NO ANGLES LEFT" << std::endl;
        rtree.insert(query);
        return;
    }

    // vertical distance filter
    results.erase(
        std::remove_if(results.begin(), results.end(), 
            [vert_thresh, centroid](const segment_type result) {
                return !verticalProximityCheck(result, centroid, vert_thresh);
            }), 
        results.end()
    );

    // for (const auto& segment : results) {
    //     // const auto& segment = result.first;
    //     std::cout << "Segment passes vertical proximity check: ("
    //               << bg::get<0, 0>(segment) << ", " << bg::get<0, 1>(segment) << ") to ("
    //               << bg::get<1, 0>(segment) << ", " << bg::get<1, 1>(segment) << ")\n";
    // }

    //ISSUE SOMETIMES THE VERTICAL DISTANCE 
    if (results.empty()) {
        std::cout << "SEGMENT INSERTED - NO VERT LEFT" << std::endl;
        rtree.insert(query);
        return;
    }
    
    // spatial relationship
    //debug check how many segments left, "should" be one
    // Access the first element of results vector
    const segment_type& remainingSegment = results.front();
    
    spatialRelation(remainingSegment, query, tau_s);

    //exit or merge + update
    //basically just want to visualize all the lines in rtree here and then later it will be like if true find spots, else, leave it? later we need to update occupancy.

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
        
        segment_type linestring(point_1, point_2);

        double length = bg::length(linestring);

        // // Process points to ensure point 1 x is smaller. Pushing into correct bounds.
        // if (point_1.get<0>() > point_2.get<0>() || (point_1.get<0>() == point_2.get<0>() && point_1.get<1>() > point_2.get<1>())) {
        //     std::swap(point_1, point_2);
        // }

        // point_type centroid((point_1.get<0>() + point_2.get<0>()) / 2, (point_1.get<1>() + point_2.get<1>()) / 2);

        // double length = std::sqrt(
        //     std::pow(point_2.get<0>() - point_1.get<0>(), 2) +
        //     std::pow(point_2.get<1>() - point_1.get<1>(), 2)
        // );  

        // double orientation = std::atan2(point_2.get<1>() - point_1.get<1>(), point_2.get<0>() - point_1.get<0>());

        // std::cout << "Point 1: (" << point_1.get<0>() << ", " << point_1.get<1>() << ")" << std::endl;
        // std::cout << "Point 2: (" << point_2.get<0>() << ", " << point_2.get<1>() << ")" << std::endl;

        // if (orientation < 0) {
        //     // std::cout << "NEGATIVE ANGLE!: " << orientation * 180 / M_PI  << std::endl;
        //     orientation += M_PI; // should this be 2pi..     
        // }

        // // std::cout << "POSITIVE ANGLE!: " << 180 * orientation / M_PI << std::endl;
        // orientation = orientation * 180 / M_PI;   

        if (length >= 1.5) {
            // rtree.insert(std::make_pair(segment_type(point_1, point_2), EndpointState{false, false, orientation}));
            queryCheck(linestring);    
            // mergedetect(); 
            // splitdetect();
            // spotdetection();
        }

        visualizeSegments(marker_pub);
        
    } else {
        ROS_WARN("Received marker with insufficient points.");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "auto_patrol");
    ros::NodeHandle nh;

    ros::Subscriber marker_sub = nh.subscribe("/line_one", 10, markerCallback);
    // ros::Subscriber marker_sub2 = nh.subscribe("/line_two", 10, markerCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker_array", 10);

    ros::spin();

    return 0;
}
