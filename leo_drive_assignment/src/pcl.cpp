#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

static std::string toString(const Eigen::MatrixXd& mat){
    std::stringstream ss;
    ss << mat;
    return ss.str();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("point_cloud_publisher");

    // pcd files path defined
    std::string pcd_file_1 = "/home/melike/leo_deneme1_ws/src/leo_drive_assignment/data/capture0001.pcd";

    std::string pcd_file_2 = "/home/melike/leo_deneme1_ws/src/leo_drive_assignment/data/capture0002.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_1, *cloud1) == -1)
    {
    RCLCPP_INFO(node->get_logger(), "PCD file 1 colud not load");
    return -1;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_2, *cloud2) == -1)
    {
    RCLCPP_INFO(node->get_logger(), "PCD file 2 colud not load");
    return -1;
    }
    //messages are defined
    sensor_msgs::msg::PointCloud2::Ptr msg1(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud1, *msg1);
    msg1->header.frame_id = "base_link"; 

    sensor_msgs::msg::PointCloud2::Ptr msg2(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud2, *msg2);
    msg2->header.frame_id = "base_link"; 
    //node created
    auto publisher1 = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_topic_1", 10);
    auto publisher2 = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_topic_2", 10);

    auto now = node->now();
    msg1->header.stamp = now;
    publisher1->publish(*msg1);
    msg2->header.stamp = now;
    publisher2->publish(*msg2);

   
    //Calculation of the transform matrix
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(20);
    gicp.setTransformationEpsilon(0.0000001);

    std::vector<int> indices;
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
    pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices2);
    gicp.setInputSource(cloud1);
    gicp.setInputTarget(cloud2);
    
    pcl::PointCloud<pcl::PointXYZ> aligned_source ;
	pcl::PointCloud<pcl::PointXYZ> aligned_source_transformed;

    gicp.align(aligned_source);
    bool is_converged = gicp.hasConverged();
    if(is_converged)
        RCLCPP_INFO(node->get_logger(),"converged");

    Eigen::Matrix<double, 4, 4> transformation = gicp.getFinalTransformation().cast<double>(); //4x4 matrix defined
    
    pcl::transformPointCloud(*cloud1, aligned_source_transformed, transformation); 
    RCLCPP_INFO(node->get_logger(),toString(transformation).c_str());

    //The point clouds to be merged were identified and combined with the addition method.
    pcl::PointCloud<pcl::PointXYZRGB> first_pc;
    pcl::PointCloud<pcl::PointXYZRGB> second_pc;
    pcl::copyPointCloud(aligned_source_transformed, first_pc);
    pcl::copyPointCloud(*cloud2, second_pc);

    //Defined colors of point clouds in RGB format
    for (size_t i = 0; i < first_pc.points.size(); i++) {
    first_pc.points[i].r = 255;
    first_pc.points[i].g = 0;
    first_pc.points[i].b = 0;
    }

    for (size_t i = 0; i < second_pc.points.size(); i++) {
    second_pc.points[i].r = 0;
    second_pc.points[i].g = 255;
    second_pc.points[i].b = 0;
    }
    first_pc += second_pc; // Combined using transform matrix of two different point clouds

    //Merged point cloud published over the node
    auto publisher3 = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_topic_3", 10);
    sensor_msgs::msg::PointCloud2::Ptr msg3(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(first_pc, *msg3);
    msg3->header.frame_id = "base_link";

    now = node->now();
    msg3->header.stamp = now;
    publisher3->publish(*msg3);

    //Point cloud converted to pcd file
    pcl::io::savePCDFileASCII ("point_cloud_3.pcd", first_pc);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
