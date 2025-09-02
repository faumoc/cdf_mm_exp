#pragma once
#include <pinocchio/fwd.hpp>
#include <controller_interface/controller.h>
#include "Types.h"
#include <hardware_interface/joint_command_interface.h>
#include "SystemObservation.h"
#include "SwerveModelInfo.hpp"
#include "SwerveTarget.hpp"

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <mm_visualize/mm_config.hpp>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Bool.h>
#include "map_util.hpp"
// #include "solver_core/SolverInterface.h"
#include <mm_msg/TargetTrajectories.h>
#include <mm_msg/SolverInput.h>

using namespace mobile_manipulator;
// using namespace CRISP;
namespace mm_ros_control {

    class mmRosControl : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        mmRosControl() {};
        ~mmRosControl() {};


        /**
         * \brief Initialize controller
         * \param hw            Velocity joint interface for the wheels
         * \param root_nh       Node handle at root namespace
         * \param controller_nh Node handle inside the controller namespace
         */

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

        /**
         * \brief Updates controller and sets the new velocity commands
         * \param time   Current time
         * \param period Time since the last called to update
         */
        void update(const ros::Time& time, const ros::Duration& period);
        void SolverThread();
        void odomCallback(const nav_msgs::OdometryPtr& msg);
        void cmdVelCallback(const geometry_msgs::TwistPtr& msg);
        void keyboardCallback(const std_msgs::Bool::ConstPtr& msg );
        void trajectoryCallback(const mm_msg::TargetTrajectories::ConstPtr& msg);

        private:

        bool firstRun_;
        
        std::vector<hardware_interface::JointHandle> steerJoints_;
        std::vector<hardware_interface::JointHandle> wheelJoints_;
        std::vector<hardware_interface::JointHandle> armJoints_;
        std::vector<hardware_interface::JointHandle> brakeJoints_;
        std::vector<hardware_interface::JointHandle> gripperJoints_;

        std::mutex updateOdomMutex_;
        SystemObservation observation_;
        SystemObservation currObservation_;
        SwerveModelInfo swerveModelInfo_;
        ModelSettings modelSettings_;
        std::unique_ptr<SwerveTarget> swerveTarget_;
        geometry_msgs::Pose currentObservedPose_;
        vector_t ee_target_;
        struct RobotInputs {
            Eigen::Vector4d wheelsInput;
            Eigen::Vector4d steersInput;
            Eigen::VectorXd armInputs;
            Eigen::Vector4d brakesInput;
            scalar_t gripperInput;
            ros::Time stamp;
            
            RobotInputs()
                : wheelsInput(Eigen::Vector4d::Zero()),
                steersInput(Eigen::Vector4d::Zero()),
                armInputs(Eigen::VectorXd::Zero(6)),
                brakesInput(Eigen::Vector4d::Zero()),
                stamp(0.0) {}
        };
        std::vector<std::string> jointNames_;
        std::unique_ptr<tf::TransformListener> listener_;
        tf::StampedTransform ee_transform_;
        realtime_tools::RealtimeBuffer<RobotInputs> robotInputs_;
        
        ros::Subscriber odometrySub_;
        ros::Subscriber subCmdVel_;
        ros::Subscriber subKeyboard_;
        ros::Subscriber subTrajectory_;
        ros::Publisher StatePublisher_;
        bool getTrajectoryFlag_ = true;
        vector_t keyCmdVel_;
        bool keyCmdMod_ = false;
        bool initTargetSet_ = false;

        MMVisConfig::Ptr mmVisConfigPtr_;
        map_util mapUtil_;
        // std::unique_ptr<SolverInterface> solver_;
        std::thread solver_thread_;
        
        std::vector<mobile_manipulator::MMState> ObjectTrajectory_;
        std::mutex trajectoryMutex_;
    };
} // namespace mmRosControl