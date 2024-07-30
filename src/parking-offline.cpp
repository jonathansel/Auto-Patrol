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

void queryCheck(const point_type& centroid, const segment_type& linestring, const double& orientation) {
    results.clear();
    
    // Define the maximum distance for nearest neighbor search in meters
    double max_distance = 0.05; // 50mm converted to meters

    // Query nearest neighbors and calculate the centroid distance
    rtree.query(bgi::nearest(centroid, 5) && bgi::satisfies([&](const std::pair<segment_type, EndpointState>& v) {
        // Calculate the centroid of the line segment
        point_type segment_centroid;
        bg::centroid(v.first, segment_centroid);
        
        std::cout << "CENTROID: " << bg::wkt(segment_centroid) << std::endl;
        // Return true if the distance between the centroids is within max_distance
        return bg::distance(segment_centroid, centroid) <= max_distance;
    }), std::back_inserter(results));
    
    // Query for intersections with the linestring
    rtree.query(bgi::intersects(linestring), std::back_inserter(results));
    
    if (results.empty()) {
        rtree.insert(std::make_pair(linestring, EndpointState{false, false, orientation}));
        std::cout << "Line inserted." << std::endl;
    } else {
        std::cout << "Intersecting or nearby lines found: " << results.size() << std::endl;
    }

    // Optional: Print the results
    // for (const auto& result : results) {
    //     std::cout << "Segment: (" << bg::wkt(result.first.first) << ", " << bg::wkt(result.first.second)
    //               << "), Endpoint State: ("
    //               << std::boolalpha << result.second.stateOne << ", "
    //               << result.second.stateTwo << ", " << result.second.orientation << ")" << std::endl;
    // }
}

void pointProcess(point_type point_1, point_type point_2) {

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
}

int main(int argc, char **argv) {
    // two points define a line
    point_type point1(-0.550185, 6.70697);
    point_type point2(-0.446606, 2.2523);  
    pointProcess(point1, point2);

    point1 = point_type(-0.570406, 6.70657);
    point2 = point_type(-0.447065, 2.25268);  
    pointProcess(point1, point2);

    return 0;
}

// Point 1: (-0.550185, 6.70697)
// Point 2: (-0.446606, 2.2523)
//   line inserted 
// Point 1: (-0.570406, 6.70657)
// Point 2: (-0.447065, 2.25268)
//   line inserted 
// Point 1: (-0.557573, 6.70561)
// Point 2: (-0.447232, 2.25246)
// Intersecting lines found: 1