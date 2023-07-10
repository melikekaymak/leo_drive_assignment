#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <GeographicLib/LocalCartesian.hpp>

std::vector<geometry_msgs::msg::PointStamped> parseKML(const std::string& kmlFilePath) 
{
    std::ifstream file(kmlFilePath);

    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> altitudes;

    if (!file.is_open()) {
        std::cerr << "Failed to open KML file" << std::endl;
        return {};
    }
    std::vector<geometry_msgs::msg::PointStamped> points;
    std::string line;
    //Detecting points as latitude, longitude and height and separating them one by one.
    while (std::getline(file, line)) 
    {
        size_t start = line.find("<coordinates>");
        size_t end = line.find("</coordinates>");
        
        if (start != std::string::npos && end != std::string::npos) 
        {
            std::string coordinatesStr = line.substr(start + 13, end - start - 13);
            size_t pos = 0;
            std::string delimiter = " ";
            while ((pos = coordinatesStr.find(delimiter)) != std::string::npos) 
            {

                std::string coord = coordinatesStr.substr(0, pos);
                size_t comma1 = coord.find(",");
                size_t comma2 = coord.find(",", comma1 + 1);

                std::string lonStr = coord.substr(0, comma1);
                std::string latStr = coord.substr(comma1 + 1, comma2 - comma1 - 1);
                std::string altStr = coord.substr(comma2 + 1);

                double lon = std::stod(lonStr); 
                double lat = std::stod(latStr);
                double alt = std::stod(altStr);

                longitudes.push_back(lon);
                latitudes.push_back(lat);
                altitudes.push_back(alt);

                coordinatesStr.erase(0, pos + delimiter.length());
            }

        }
    } 
    
    file.close();
    //Converting the separated coordinates to the local coordinate system.
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    GeographicLib::LocalCartesian proj(latitudes[0], longitudes[0], altitudes[0], earth);
    
    double originLat = latitudes[0];
    double originLon = longitudes[0];
    double originAlt = altitudes[0];
    proj.Reset(originLat, originLon, originAlt);

    std::vector<double> localX(latitudes.size());
    std::vector<double> localY(latitudes.size());
    std::vector<double> localZ(latitudes.size());
    
        

    for (size_t i = 1; i < latitudes.size(); ++i) {
        double lat = latitudes[i];
        double lon = longitudes[i];
        double alt = altitudes[i];
        double x, y, z;
        proj.Forward(lat, lon, alt, x, y, z);

        localX.push_back(x);
        localY.push_back(y);
        localZ.push_back(z);
        geometry_msgs::msg::PointStamped pose;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        pose.point.x = x;
        pose.point.y = y;
        pose.point.z = z;
        points.push_back(pose);
        
    }

    
    return points;
    }
//Create a node and publish it as nav_msgs
int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kml_parser_node");

    std::string kmlFilePath = "/home/melike/leo_deneme1_ws/src/leo_drive_assignment/data/test_route.kml";
    std::vector<geometry_msgs::msg::PointStamped> points = parseKML(kmlFilePath);

    
    auto pathMsg = std::make_shared<nav_msgs::msg::Path>();
    pathMsg->header.stamp = rclcpp::Clock().now();
    pathMsg->header.frame_id = "map"; 
    pathMsg->poses.resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = pathMsg->header;
        pose.pose.position.x = points[i].point.x;
        pose.pose.position.y = points[i].point.y;
        pose.pose.position.z = points[i].point.z;
        pathMsg->poses.push_back(pose);
    }


    auto pathPub = node->create_publisher<nav_msgs::msg::Path>("kml_path", 10);
    pathPub->publish(*pathMsg);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
