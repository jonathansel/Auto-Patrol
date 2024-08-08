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
// std::vector<std::pair<segment_type, EndpointState>> segments;
std::vector<std::pair<segment_type, EndpointState>> results;

using rtree_type = bgi::rtree<std::pair<segment_type, EndpointState>, bgi::linear<16>>;
rtree_type rtree;

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
bool angularProximityCheck(const segment_type& seg1, const segment_type& seg2, const double& tau_theta) {
    auto direction1 = directionVector(seg1.first, seg1.second);
    auto direction2 = directionVector(seg2.first, seg2.second);

    double cross_prod = crossProduct(direction1, direction2);
    double mag1 = magnitude(direction1);
    double mag2 = magnitude(direction2);

    double angular_proximity = std::abs(cross_prod) / (mag1 * mag2);
    return angular_proximity < std::sin(tau_theta);
}

bool verticalProximityCheck(const segment_type& seg1, const point_type& centroid, const double& vert_thresh) {
    //just calc centroid here?
    auto direction1 = directionVector(seg1.first, centroid); //P1QM
    auto direction2 = directionVector(seg1.first, seg1.second); //P1P2

    double cross_prod = crossProduct(direction1, direction2);
    double mag2 = magnitude(direction2);
    double vertical_proximity = std::abs(cross_prod) / (mag2);
    
    return vertical_proximity < vert_thresh;
}

// seg1 = result (P1P2) seg2 = query (Q1Q2)
void spatialRelation(const segment_type& seg1, const segment_type& seg2) {
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

    std::cout << "max distance: " << max_distance << std::endl;

    auto direction1 = directionVector(seg1.first, seg1.second);//p1p2
    auto direction2 = directionVector(seg1.first, seg2.first); //p1q1
    auto direction3 = directionVector(seg1.first, seg2.second); //p1q2
    double mag1 = magnitude(direction1); //mag(p1p2)
    double tau_s = 0.5 * mag1;

    //projection q1'
    double q1_proj = dotProduct(direction2, direction1) / (mag1);
    double q2_proj = dotProduct(direction3, direction1) / (mag1);

    double q1q2_mag = std::abs(q1_proj - q2_proj);
    std::cout << "q1q2 mag: " << q1q2_mag << std::endl;
    double merge_criteria = mag1 + q1q2_mag; 
    
    if (max_distance == mag1) {
            std::cout << "P1P2 best option, return" << std::endl;
    }

    if (max_distance - merge_criteria <= tau_s) {
        std::cout << "non-overlapping at suitable distance or partial overlapping" << std::endl;
        rtree.remove(std::make_pair(seg1, EndpointState{false, false, 90}));
        rtree.insert(std::make_pair(longest_segment, EndpointState{false, false, 90}));
    }
    
        // Output the initial segments
    // std::cout << "Remaining segments in the R-tree:" << std::endl;
    // for (const auto& seg : rtree) {
    //     std::cout << "Segment: ((" << bg::get<0, 0>(seg.first) << ", " << bg::get<0, 1>(seg.first) << "), ("
    //               << bg::get<1, 0>(seg.first) << ", " << bg::get<1, 1>(seg.first) << "))" << std::endl;
    // }

}

void queryCheck(const point_type& centroid, const segment_type& query) {
    results.clear();
    
    // Define the maximum distance for nearest neighbor search in meters - tune
    double vert_thresh = 3; // 3mm
    double tau_theta = 0.0523599; // 3 degrees in rad

    rtree.query(bgi::nearest(centroid, 5), std::back_inserter(results)); //change to linestring

    // Remove segments that do not pass the angular proximity check
    results.erase(
        std::remove_if(results.begin(), results.end(), 
            [&query, tau_theta](const std::pair<segment_type, EndpointState>& result) {
                return !angularProximityCheck(result.first, query, tau_theta);
            }), 
        results.end()
    );

    // Loop through the remaining segments that passed the angular proximity check
    for (const auto& result : results) {
        const auto& segment = result.first;
        std::cout << "Segment passes angular proximity check: ("
                  << bg::get<0, 0>(segment) << ", " << bg::get<0, 1>(segment) << ") to ("
                  << bg::get<1, 0>(segment) << ", " << bg::get<1, 1>(segment) << ")\n";
    }

    // spatial relationship
    if (results.empty()) {
        std::cout << "No segments left after angle proximity checks." << std::endl;
        return;
    }

    // vertical distance filter
    results.erase(
        std::remove_if(results.begin(), results.end(), 
            [vert_thresh, centroid](const std::pair<segment_type, EndpointState>& result) {
                return !verticalProximityCheck(result.first, centroid, vert_thresh);
            }), 
        results.end()
    );

    for (const auto& result : results) {
        const auto& segment = result.first;
        std::cout << "Segment passes vertical proximity check: ("
                  << bg::get<0, 0>(segment) << ", " << bg::get<0, 1>(segment) << ") to ("
                  << bg::get<1, 0>(segment) << ", " << bg::get<1, 1>(segment) << ")\n";
    }

    // spatial relationship
    if (results.empty()) {
        std::cout << "No segments left after vertical distance checks." << std::endl;
        return;
    }
    
    // Access the first element of results vector
    const segment_type& remainingSegment = results.front().first;
    

    spatialRelation(remainingSegment, query);


    // Find the longest line (l_r) and the points forming it
    // segment_type longestLine = findLongestLine(query, remainingSegment);



    // Output the result
    // std::cout << "Longest line (l_r) length: " << euclideanDistance(longestLine.first, longestLine.second) << std::endl;
    // std::cout << "Points forming the longest line: ("
    //           << bg::get<0>(longestLine.first) << ", " << bg::get<1>(longestLine.first) << ") to ("
    //           << bg::get<0>(longestLine.second) << ", " << bg::get<1>(longestLine.second) << ")" << std::endl;


    //exit or merge + update



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

    if (length >= 1.0) {
        // rtree.insert(std::make_pair(segment_type(point_1, point_2), EndpointState{false, false, orientation}));
        queryCheck(centroid, segment_type(point_1, point_2));
        // mergedetect(); 
        // splitdetect();
        // spotdetection();
    }
}

int main(int argc, char **argv) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // two points define a line
    point_type point1(0, 0);
    point_type point2(0, 70);  
    rtree.insert(std::make_pair(segment_type(point1, point2), EndpointState{false, false, 90}));

    point1 = point_type(50, 0);
    point2 = point_type(50, 70);  
    rtree.insert(std::make_pair(segment_type(point1, point2), EndpointState{false, false, 90}));

    point1 = point_type(100, 10);
    point2 = point_type(100, 70);  
    rtree.insert(std::make_pair(segment_type(point1, point2), EndpointState{false, false, 90}));

    point1 = point_type(150, 0);
    point2 = point_type(150, 70);  
    rtree.insert(std::make_pair(segment_type(point1, point2), EndpointState{false, false, 90}));

    point1 = point_type(0, 70);
    point2 = point_type(200, 70);  
    rtree.insert(std::make_pair(segment_type(point1, point2), EndpointState{false, false, 0}));

    // rtree.query(bgi::nearest(point_type(4, 5), 3), std::back_inserter(results));

    // non-overlapping
    point1 = point_type(102, 8);
    point2 = point_type(102, -12);  

    // fully overlapping
    // point1 = point_type(102, 50);
    // point2 = point_type(102, 25);  

    // longer than original line
    // point1 = point_type(102, -10);
    // point2 = point_type(102, 80);  

    // point1 = point_type(102, 8);
    // point2 = point_type(102, -10);  


    segment_type linestring(point1, point2);

    point_type centroid((point1.get<0>() + point2.get<0>()) / 2, (point1.get<1>() + point2.get<1>()) / 2);

    queryCheck(centroid, linestring);

    // Print the results
    // for (const auto& result : results) {
    // std::cout << "Segment: (" << bg::wkt(result.first.first) << ", " << bg::wkt(result.first.second) << "), Endpoint State: ("
    //             << std::boolalpha << result.second.stateOne << ", " << result.second.stateTwo << ", " << result.second.orientation << ")" << std::endl;
    // }

    // Stop timing
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;
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
    // point_type point1(-0.550185, 6.70697);
    // point_type point2(-0.446606, 2.2523);  
    // pointProcess(point1, point2);

    // point1 = point_type(-0.570406, 6.70657);
    // point2 = point_type(-0.447065, 2.25268);  
    // pointProcess(point1, point2);

    // return 0;

        // Query nearest neighbors and calculate the centroid distance
    // rtree.query(bgi::nearest(centroid, 5) && bgi::satisfies([&](const std::pair<segment_type, EndpointState>& v) {
    //     // Calculate the centroid of the line segment
    //     point_type segment_centroid;
    //     bg::centroid(v.first, segment_centroid);
        
    //     std::cout << "CENTROID: " << bg::wkt(segment_centroid) << std::endl;
    //     // Return true if the distance between the centroids is within max_distance
    //     return bg::distance(segment_centroid, centroid) <= max_distance;
    // }), std::back_inserter(results));
    
    // // Query for intersections with the linestring
    // rtree.query(bgi::intersects(linestring), std::back_inserter(results));
    
    // if (results.empty()) {
    //     rtree.insert(std::make_pair(linestring, EndpointState{false, false, orientation}));
    //     std::cout << "Line inserted." << std::endl;
    // } else {
    //     std::cout << "Intersecting or nearby lines found: " << results.size() << std::endl;
    // }

        // Loop through the found nearest segments and check angular proximity
    // for (const auto& result : results) {
    //     if (angularProximityCheck(query, result.first, tau_theta)) {
    //         // Handle segments that pass the angular proximity check
    //         std::cout << "Segment passes angular proximity check: ("
    //                   << bg::get<0, 0>(result.first) << ", " << bg::get<0, 1>(result.first) << ") to ("
    //                   << bg::get<1, 0>(result.first) << ", " << bg::get<1, 1>(result.first) << ")\n";
    //     } else {
    //         std::cout << "Segment does not pass angular proximity check: ("
    //                   << bg::get<0, 0>(result.first) << ", " << bg::get<0, 1>(result.first) << ") to ("
    //                   << bg::get<1, 0>(result.first) << ", " << bg::get<1, 1>(result.first) << ")\n";
    //     }
    // }

    // Optional: Print the results
    // for (const auto& result : results) {
    //     std::cout << "Segment: (" << bg::wkt(result.first.first) << ", " << bg::wkt(result.first.second)
    //               << "), Endpoint State: ("
    //               << std::boolalpha << result.second.stateOne << ", "
    //               << result.second.stateTwo << ", " << result.second.orientation << ")" << std::endl;
    // }