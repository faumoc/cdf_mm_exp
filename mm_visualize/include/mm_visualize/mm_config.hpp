#pragma once

#include "pinocchio/fwd.hpp"
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace mobile_manipulator
{
    struct MMState{
        double time_stamp;
        
        Eigen::VectorXd q;
        Eigen::VectorXd dq;
    };

    class MMVisConfig
    {
    
    public:
        MMVisConfig();
        ~MMVisConfig() {}

        void DisplayTrajectory(const std::vector<MMState> &traj, double alpha);
        visualization_msgs::MarkerArray getMarkerArray(const Eigen::VectorXd &joint_state, int idx, double alpha);
        visualization_msgs::Marker getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file);

        void setParam(ros::NodeHandle &nh);
        void setColorSet();

    private:
        std::vector<Eigen::Vector3d> color_set_;

        std::string mesh_mobile_base_, mesh_arm_base_, mesh_arm_link1_, mesh_arm_link2_, mesh_arm_link3_, mesh_arm_link4_, mesh_arm_link5_, mesh_arm_link6_;
        // pinocchio model ptr
        std::shared_ptr<pinocchio::Model> pinocchio_model_ptr_;
        std::shared_ptr<pinocchio::Data> pinocchio_data_ptr_;
        ros::Publisher traj_vis_pub_;
        int vis_idx_size_ ;
    public:
        // typedef std::unique_ptr<MMVisConfig> Ptr;
        typedef std::shared_ptr<MMVisConfig> Ptr;
    };

    
} // namespace remani_planner


