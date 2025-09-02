#include <pinocchio/fwd.hpp>
#include <mm_visualize/mm_config.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
namespace mobile_manipulator
{
MMVisConfig::MMVisConfig(): mesh_mobile_base_(""),
          mesh_arm_base_(""),
          mesh_arm_link1_(""),
          mesh_arm_link2_(""),
          mesh_arm_link3_(""),
          mesh_arm_link4_(""),
          mesh_arm_link5_(""),
          mesh_arm_link6_(""),
          vis_idx_size_(0) {};

void MMVisConfig::setParam(ros::NodeHandle &nh){
     
    std::string mesh_path = ros::package::getPath("swerve_description")  + "/meshes/";
    

    mesh_mobile_base_ = "file://" + mesh_path + "holonomic/base_link.STL";
    mesh_arm_base_ = "file://" + mesh_path + "jaco/base_link.STL";
    mesh_arm_link1_ = "file://" + mesh_path + "jaco/shoulder_link.STL";
    mesh_arm_link2_ = "file://" + mesh_path + "jaco/bicep_link.STL";
    mesh_arm_link3_ = "file://" + mesh_path + "jaco/forearm_link.STL";
    mesh_arm_link4_ = "file://" + mesh_path + "jaco/spherical_wrist_1_link.STL";
    mesh_arm_link5_ = "file://" + mesh_path + "jaco/spherical_wrist_2_link.STL";
    mesh_arm_link6_ = "file://" + mesh_path + "jaco/bracelet_with_vision_link.STL";
    
    const std::string urdf_path = ros::package::getPath("swerve_description") + "/urdf/holonomic_mm_arm.urdf.xacro";

    std::string urdfString;
    nh.getParam("/swerve_mpc/robot_description", urdfString);
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(urdfString);
    if (urdfTree != nullptr)
    {
        ROS_INFO_STREAM("Successfully parsed URDF from parameter server.");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to parse URDF from parameter server.");
        return;
    }

    pinocchio_model_ptr_ = std::make_shared<pinocchio::Model>();
    pinocchio::JointModelFreeFlyerTpl<double> rootJoint;
    pinocchio::urdf::buildModel(urdfTree, rootJoint, *pinocchio_model_ptr_, true);
    pinocchio_data_ptr_ = std::make_shared<pinocchio::Data>(*pinocchio_model_ptr_);

    vis_idx_size_ = 10; // 8 links in the mobile manipulator
    
    traj_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traj_vis", 50);
    setColorSet();
}

void MMVisConfig::setColorSet(){
    color_set_.clear();
    // red
    Eigen::Vector3d color;
    color << 255, 31, 91;
    color /= 255.0;
    color_set_.push_back(color);
    // green
    color << 0, 205, 108;
    color /= 255.0;
    color_set_.push_back(color);
    // blue
    color << 0, 154, 222;
    color /= 255.0;
    color_set_.push_back(color);
    // yellow
    color << 255, 198, 30;
    color /= 255.0;
    color_set_.push_back(color);
    // grey
    color << 160, 177, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // orange
    color << 234, 96, 22;
    color /= 255.0;
    color_set_.push_back(color);
    // purple
    color << 175, 88, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // brown
    color << 166, 118, 29;
    color /= 255.0;
    color_set_.push_back(color);
}

void MMVisConfig::DisplayTrajectory(const std::vector<MMState> &traj, double alpha) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < traj.size(); ++i) {
        auto marker = getMarkerArray(traj[i].q, i, alpha);
        marker_array.markers.insert(marker_array.markers.end(), marker.markers.begin(), marker.markers.end());
        // std::cout << "Displaying trajectory: " << i << std::endl;
        // std::cout << "Joint state: " << traj[i].q.transpose() << std::endl;
    }
    traj_vis_pub_.publish(marker_array);
}

visualization_msgs::MarkerArray MMVisConfig::getMarkerArray(const Eigen::VectorXd &joint_state, int idx, double alpha) {
    pinocchio::forwardKinematics(*pinocchio_model_ptr_, *pinocchio_data_ptr_, joint_state);
    pinocchio::updateFramePlacements(*pinocchio_model_ptr_, *pinocchio_data_ptr_);

    int indBaseLink = pinocchio_model_ptr_->getFrameId("base_link");
    int indXarmLink = pinocchio_model_ptr_->getFrameId("x_arm_link");
    int indShoulder = pinocchio_model_ptr_->getFrameId("shoulder_link");
    int indBicepLink = pinocchio_model_ptr_->getFrameId("bicep_link");
    int indForearmLink = pinocchio_model_ptr_->getFrameId("forearm_link");
    int indWrist_1Link = pinocchio_model_ptr_->getFrameId("spherical_wrist_1_link");
    int indWrist_2Link = pinocchio_model_ptr_->getFrameId("spherical_wrist_2_link");
    int indBraceletLink = pinocchio_model_ptr_->getFrameId("bracelet_link");

    auto &data = *pinocchio_data_ptr_;
    visualization_msgs::MarkerArray marker_array;
    Eigen::Matrix4d worldToBaseLinkTransf = data.oMf[indBaseLink].toHomogeneousMatrix();
    Eigen::Matrix4d worldToXarmLinkTransf = data.oMf[indXarmLink].toHomogeneousMatrix();
    Eigen::Matrix4d worldToShoulderTransf = data.oMf[indShoulder].toHomogeneousMatrix();
    Eigen::Matrix4d worldToBicepLinkTransf = data.oMf[indBicepLink].toHomogeneousMatrix();
    Eigen::Matrix4d worldToForearmLinkTransf = data.oMf[indForearmLink].toHomogeneousMatrix();
    Eigen::Matrix4d worldToWrist_1LinkTransf = data.oMf[indWrist_1Link].toHomogeneousMatrix();
    Eigen::Matrix4d worldToWrist_2LinkTransf = data.oMf[indWrist_2Link].toHomogeneousMatrix();
    Eigen::Matrix4d worldToBraceletLinkTransf = data.oMf[indBraceletLink].toHomogeneousMatrix();

    marker_array.markers.push_back(getMarker(idx*vis_idx_size_, "traj_vis", alpha, worldToBaseLinkTransf, mesh_mobile_base_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 1, "traj_vis", alpha, worldToXarmLinkTransf, mesh_arm_base_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 2, "traj_vis", alpha, worldToShoulderTransf, mesh_arm_link1_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 3, "traj_vis", alpha, worldToBicepLinkTransf, mesh_arm_link2_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 4, "traj_vis", alpha, worldToForearmLinkTransf, mesh_arm_link3_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 5, "traj_vis", alpha, worldToWrist_1LinkTransf, mesh_arm_link4_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 6, "traj_vis", alpha, worldToWrist_2LinkTransf, mesh_arm_link5_));
    marker_array.markers.push_back(getMarker(idx*vis_idx_size_ + 7, "traj_vis", alpha, worldToBraceletLinkTransf, mesh_arm_link6_));

    return marker_array;
    }
    // Publish the MarkerArray
    // publisher.publish(marker_array); // Uncomment this line to publish the markers

visualization_msgs::Marker MMVisConfig::getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file){
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::Marker meshMarker;
    meshMarker.header.frame_id = "odom";
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = false;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    if(alpha >= 0.0)
        meshMarker.color.a = alpha;
    meshMarker.color.r = 0.75294;
    meshMarker.color.g = 0.75294;
    meshMarker.color.b = 0.75294;
    meshMarker.mesh_resource = mesh_file;
    return meshMarker;
}

}// namespace mobile_manipulator
