#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace mm_ros_control{
using Vec3f = Eigen::Matrix<double , 3, 1>;
class map_util
{
public:
    map_util(){};
    map_util(ros::NodeHandle nh);
    ~map_util(){};
    pcl::PointCloud<pcl::PointXYZ> readPCDFile(const std::string& file_name);
    void cropPointCloud( Vec3f& center, float radius, pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub;
    pcl::PointCloud<pcl::PointXYZ> cloud_;  // Store the point cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_;  // Store the filtered point cloud data
    /* data */
};

}
