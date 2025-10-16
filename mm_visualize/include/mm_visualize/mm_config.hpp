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
    using scalar_t = double;
    using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
    struct MMState{
        double time_stamp;
        
        Eigen::VectorXd q;
        Eigen::VectorXd state;
        Eigen::VectorXd input;
    };

    class MMVisConfig
    {
    
    public:
        MMVisConfig();
        ~MMVisConfig() {}

        void DisplayTrajectory(const std::vector<MMState> &traj, double alpha);
        visualization_msgs::MarkerArray getMarkerArray(const Eigen::VectorXd &joint_state, int idx, double alpha);
        visualization_msgs::Marker getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file);
        void UpdatePinocchioModel(const vector_t& joint_state);
        vector_t baseKinemics(const vector_t& baseInput);
        void setParam(ros::NodeHandle &nh);
        void setColorSet();
        void setUpperBound(const vector_t& upperBound){
            upperBound_ = upperBound;
        }

        

    private:
        std::vector<Eigen::Vector3d> color_set_;

        std::string mesh_mobile_base_, mesh_arm_base_, mesh_arm_link1_, mesh_arm_link2_, mesh_arm_link3_, mesh_arm_link4_, mesh_arm_link5_, mesh_arm_link6_;
        // pinocchio model ptr
        std::shared_ptr<pinocchio::Model> pinocchio_model_ptr_;
        std::shared_ptr<pinocchio::Data> pinocchio_data_ptr_;
        ros::Publisher traj_vis_pub_;
        int vis_idx_size_ ;
        vector_t currState_;
        vector_t upperBound_;
        double KpSteer_ = 5;
    public:
        // typedef std::unique_ptr<MMVisConfig> Ptr;
        typedef std::shared_ptr<MMVisConfig> Ptr;
        
    };
    double normalizeAngle(double angle);

    enum InputIndex {
        x_input_ind,
        y_input_ind,
        theta_input_ind,
        lb_steer_input_ind,
        lb_wheel_input_ind,
        lf_steer_input_ind,
        lf_wheel_input_ind,
        rb_steer_input_ind,
        rb_wheel_input_ind,
        rf_steer_input_ind,
        rf_wheel_input_ind,
        sh_rot_input_ind,
        sh_fle_input_ind,
        el_fle_input_ind,
        el_rot_input_ind,
        wr_fle_input_ind,
        wr_rot_input_ind,
        };

        enum StateIndex {
        x_state_ind,
        y_state_ind,
        z_state_ind,
        x_quat_state_ind,
        y_quat_state_ind,
        z_quat_state_ind,
        w_quat_state_ind,
        lb_steer_state_ind,
        lb_wheel_state_ind,
        lf_steer_state_ind,
        lf_wheel_state_ind,
        rb_steer_state_ind,
        rb_wheel_state_ind,
        rf_steer_state_ind,
        rf_wheel_state_ind,
        sh_rot_state_ind,
        sh_fle_state_ind,
        el_fle_state_ind,
        el_rot_state_ind,
        wr_fle_state_ind,
        wr_rot_state_ind,
        };

    
} // namespace remani_planner


