#include "map_util.hpp"
#include <crop_sphere.hpp>
namespace mm_ros_control{
map_util::map_util(ros::NodeHandle nh) : nh_(nh) {
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
}

pcl::PointCloud<pcl::PointXYZ>  map_util::readPCDFile(const std::string& file_name) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
    }
    for (auto& point : cloud.points) {
        point.x -= 2.0;
    }
    cloud_ = cloud;

    sensor_msgs::PointCloud ROS_cloud;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    sensor_msgs::convertPointCloud2ToPointCloud(output, ROS_cloud);
    ROS_cloud.header.frame_id = "odom";
    cloud_pub.publish(ROS_cloud);
    
    return cloud;
}

void map_util::cropPointCloud(Vec3f& center, float radius, pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    CropSphere<pcl::PointXYZ> crop_sphere;
    crop_sphere.setInputCloud(cloud.makeShared());
    Eigen::Vector4f center_pt(center[0], center[1], center[2], 1.0);
    crop_sphere.setCenter(center_pt);
    crop_sphere.setRadius(radius);
    crop_sphere.filter(*cloud_filtered);
    cloud = *cloud_filtered;

    sensor_msgs::PointCloud ROS_cloud;
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_filtered, output);
    sensor_msgs::convertPointCloud2ToPointCloud(output, ROS_cloud);
    ROS_cloud.header.frame_id = "odom";
    cloud_pub.publish(ROS_cloud);
    cloud_filtered_ = *cloud_filtered;  // Update the class member cloud with the filtered cloud
}
} // namespace mm_ros_control