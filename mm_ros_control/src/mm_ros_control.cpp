#include "mm_ros_control/mm_ros_control.hpp"
#include "map_util.hpp"
#include "LoadData.h"
#include <pluginlib/class_list_macros.h>
#include <mm_visualize/mm_config.hpp>
#include <mm_msg/TargetTrajectories.h>
#include <mm_msg/SolverInput.h>
namespace mm_ros_control{

bool mmRosControl::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    // Initialize the controller here
    typedef hardware_interface::VelocityJointInterface VelIface;
    const std::string complete_ns = controller_nh.getNamespace();

    std::size_t id = complete_ns.find_last_of("/");
    std::string name_ = complete_ns.substr(id + 1);

    firstRun_ = true;

    std::string taskFile, urdfString;
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/robot_description", urdfString);
    
    std::vector<std::string> steerJointNames = controller_nh.param("steer_joint_names", std::vector<std::string>());
    std::vector<std::string> wheelJointNames = controller_nh.param("wheel_joint_names", std::vector<std::string>());
    // std::vector<std::string> brakeJointNames = controller_nh.param("brake_joint_names", std::vector<std::string>());
    std::vector<std::string> armJointNames = controller_nh.param("arm_joint_names", std::vector<std::string>());
    std::vector<std::string> gripperJointNames = controller_nh.param("gripper_joint_names", std::vector<std::string>());
    
     
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

     // load variables
    loadData::loadPtreeValue(pt, swerveModelInfo_.armPresent, "model_information.armPresent", true);
    loadData::loadPtreeValue(pt, swerveModelInfo_.eeFrame, "model_information.eeFrame", true);

    swerveModelInfo_.stateDim = swerveModelInfo_.steerDim + swerveModelInfo_.wheelDim  + swerveModelInfo_.armDim + 7;
    swerveModelInfo_.inputDim = swerveModelInfo_.steerDim + swerveModelInfo_.wheelDim  + swerveModelInfo_.armDim + 3;
    swerveModelInfo_.pinnochioInputDim =
        swerveModelInfo_.steerDim + swerveModelInfo_.wheelDim  + swerveModelInfo_.armDim + 6;
    swerveModelInfo_.jointLimitsDim = swerveModelInfo_.steerDim + swerveModelInfo_.armDim;
    
    vector_t upperBound(swerveModelInfo_.inputDim);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound", upperBound);
    
    vector_t joint_upperLimit(swerveModelInfo_.jointLimitsDim);
    loadData::loadEigenMatrix(taskFile, "jointLimits.upperLimit", joint_upperLimit);
    // initial state
    vector_t initialState = Eigen::VectorXd::Zero(swerveModelInfo_.stateDim);
    loadData::loadEigenMatrix(taskFile, "initialState", initialState);
    observation_.state.resize(swerveModelInfo_.stateDim);
    observation_.state = initialState;
    observation_.input.setZero(swerveModelInfo_.inputDim);
    observation_.time = ros::Time().now().toSec();
    
    currObservation_ = observation_;

    // Initialize SwerveTarget
    swerveTarget_.reset(new SwerveTarget(root_nh, swerveModelInfo_, "odom"));
    swerveTarget_->initInterectiveMarker();
    
    mmVisConfigPtr_ = std::make_shared<MMVisConfig>();
    mmVisConfigPtr_->setParam(controller_nh);
    // Initialize tf listener
    listener_.reset(new tf::TransformListener);
    std::string odomTopic = controller_nh.param<std::string>("odom_topic", "odometry_controller/odom");
    odometrySub_ = root_nh.subscribe(odomTopic, 10, &mmRosControl::odomCallback, this);

    subCmdVel_ = root_nh.subscribe("/cmd_vel", 1, &mmRosControl::cmdVelCallback, this);
    subKeyboard_ = root_nh.subscribe("/keyboard", 1, &mmRosControl::keyboardCallback, this);
    keyCmdVel_ = vector_t::Zero(3);
    
    mapUtil_ = map_util(root_nh);
    std::string  pcd_file;
    controller_nh.getParam("/pcd_file", pcd_file);
    mapUtil_.readPCDFile(pcd_file);
    
    jointNames_ = steerJointNames;
    jointNames_.insert(jointNames_.end(), wheelJointNames.begin(), wheelJointNames.end());
    if (swerveModelInfo_.armPresent) {
        jointNames_.insert(jointNames_.end(), armJointNames.begin(), armJointNames.end());
        jointNames_.insert(jointNames_.end(), gripperJointNames.begin(), gripperJointNames.end());
    }
    for (const auto& jointName : steerJointNames) {
        steerJoints_.push_back(hw->getHandle(jointName));
        ROS_INFO_STREAM_NAMED(name_, "Adding steering velocity joint: " << jointName);
    }

    for (const auto& jointName : wheelJointNames) {
        wheelJoints_.push_back(hw->getHandle(jointName));
        ROS_INFO_STREAM_NAMED(name_, "Adding wheel velocity joint: " << jointName);
    }

    if (swerveModelInfo_.armPresent) {
        for (const auto& jointName : armJointNames) {
        armJoints_.push_back(hw->getHandle(jointName));
        ROS_INFO_STREAM_NAMED(name_, "Adding arm velocity joint: " << jointName);
        }
        for (const auto& jointName : gripperJointNames) {
        gripperJoints_.push_back(hw->getHandle(jointName));
        ROS_INFO_STREAM_NAMED(name_, "Adding gripper velocity joint: " << jointName);
        }
    }

    ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization");

    subTrajectory_ = root_nh.subscribe("mobile_manipulator_trajectory", 10, &mmRosControl::trajectoryCallback, this);
    StatePublisher_ = root_nh.advertise<mm_msg::SolverInput>("mobile_manipulator_state", 10);
    return true;
}


void mmRosControl::update(const ros::Time& time, const ros::Duration& period) {

    static std::vector<double>  arm_position(6, 0);
    static bool elapsed_time_flag = false;
    {
        std::lock_guard<std::mutex> lock(updateOdomMutex_);
        currObservation_.time = ros::Time().now().toSec();
        currObservation_.state(x_state_ind) = currentObservedPose_.position.x;
        currObservation_.state(y_state_ind) = currentObservedPose_.position.y;
        currObservation_.state(z_state_ind) = currentObservedPose_.position.z;

        currObservation_.state(x_quat_state_ind) = currentObservedPose_.orientation.x;
        currObservation_.state(y_quat_state_ind) = currentObservedPose_.orientation.y;
        currObservation_.state(z_quat_state_ind) = currentObservedPose_.orientation.z;
        currObservation_.state(w_quat_state_ind) = currentObservedPose_.orientation.w;

        for (size_t i = 0; i < steerJoints_.size(); ++i) {
        currObservation_.state(modelSettings_.steersStateIndex[i]) = steerJoints_[i].getPosition();
        }
        for (size_t i = 0; i < wheelJoints_.size(); ++i) {
            currObservation_.state(modelSettings_.wheelsStateIndex[i]) = wheelJoints_[i].getPosition();
        }
        if (swerveModelInfo_.armPresent) {
            for (size_t i = 0; i < armJoints_.size(); ++i) {
            currObservation_.state(modelSettings_.armJointsStateIndex[i]) = armJoints_[i].getPosition();
            }
        }
    }

    if (firstRun_) {
        try {
            listener_->lookupTransform("odom", swerveModelInfo_.eeFrame, ros::Time(0), ee_transform_);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        firstRun_ = false;

        vector_t initTarget(7);
        initTarget.head(3) << ee_transform_.getOrigin().x(), ee_transform_.getOrigin().y(), ee_transform_.getOrigin().z();
        initTarget.tail(4).head(4) << ee_transform_.getRotation().x(), ee_transform_.getRotation().y(), ee_transform_.getRotation().z(),
            ee_transform_.getRotation().w();
        
        ee_target_ = initTarget;
        solver_thread_ = std::thread(&mmRosControl::SolverThread, this);
        getTrajectoryFlag_ = true;

    }
    



    RobotInputs currInputs = *(robotInputs_.readFromRT());
    
    auto currtime = ros::Time::now();
    auto elapsed = currtime - currInputs.stamp;
    if (elapsed.toSec() > 1) {
    currInputs.wheelsInput.setZero();
    currInputs.steersInput.setZero();
    currInputs.armInputs.setZero();
    currInputs.gripperInput = 0;
    if (!elapsed_time_flag){
    arm_position = std::vector<double>(currObservation_.state.tail(6).data(), 
                                currObservation_.state.tail(6).data() + 6);

    elapsed_time_flag = true;
    std::cout << "\033[31mNo input received for 0.3s\033[0m" << std::endl;
    }
    for (int i = 0; i < 6; i++){
        currInputs.armInputs(i) = 0;
    // currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
    }}
    else{
        elapsed_time_flag = false;
    }

    if(1){
        std::cout << "Publishing state message..." << std::endl;
        getTrajectoryFlag_ = false;
        // Publish the current state to the topic
        mm_msg::SolverInput stateMsg;
        stateMsg.x0.data[0] = currObservation_.state(x_state_ind);
        stateMsg.x0.data[1] = currObservation_.state(y_state_ind);

        double theta_z = atan2(currentObservedPose_.orientation.z, currentObservedPose_.orientation.w);
        stateMsg.x0.data[2] = theta_z;
        stateMsg.x0.data[3] = currObservation_.state(sh_rot_state_ind);
        stateMsg.x0.data[4] = currObservation_.state(sh_rot_state_ind + 1);
        stateMsg.x0.data[5] = currObservation_.state(sh_rot_state_ind + 2);
        stateMsg.x0.data[6] = currObservation_.state(sh_rot_state_ind + 3);
        stateMsg.x0.data[7] = currObservation_.state(sh_rot_state_ind + 4);
        stateMsg.x0.data[8] = currObservation_.state(sh_rot_state_ind + 5);

        stateMsg.xf.data[0] = 3; // target x position
        StatePublisher_.publish(stateMsg);
        

        if(ObjectTrajectory_.size() > 0){
            std::lock_guard<std::mutex> lock(trajectoryMutex_);
            mmVisConfigPtr_->DisplayTrajectory(ObjectTrajectory_, 0.5);  
    } }

    if (!initTargetSet_){
        currInputs.wheelsInput.setZero();
        currInputs.steersInput.setZero();
        currInputs.armInputs.setZero();
        arm_position = std::vector<double>{0.0, -0.5, -1, 0.0, -1.1, 0.0};
        for (int i = 0; i < 6; i++)
        {
        currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
        }
    }
   
    for (size_t i = 0; i < steerJoints_.size(); ++i) {
        steerJoints_[i].setCommand(currInputs.steersInput(i));
    }
    for (size_t i = 0; i < wheelJoints_.size(); ++i) {
        wheelJoints_[i].setCommand(currInputs.wheelsInput(i));
    }
    if (swerveModelInfo_.armPresent) {
    for (size_t i = 0; i < armJoints_.size(); ++i) {
        armJoints_[i].setCommand(currInputs.armInputs(i));
    }
    for (size_t i = 0; i < gripperJoints_.size(); ++i) {
        gripperJoints_[i].setCommand(currInputs.gripperInput);
    }
    }


}

void mmRosControl::SolverThread() {

    // auto start_time = ros::Time::now();
    // solver_->solve();
    // auto xOptimal = solver_->getSolution();
    // printf("Optimal solution: \n");
    // std::cout << xOptimal.transpose() << std::endl;
    // auto end_time = ros::Time::now();
    // std::cout << "Solver took: " << (end_time - start_time).toSec() << " seconds." << std::endl;
    // auto elapsed_time = end_time - start_time;
}

void mmRosControl::odomCallback(const nav_msgs::OdometryPtr& msg) {
  std::lock_guard<std::mutex> lock(updateOdomMutex_);
  currentObservedPose_.orientation = msg->pose.pose.orientation;
  currentObservedPose_.position.x = msg->pose.pose.position.x;
  currentObservedPose_.position.y = msg->pose.pose.position.y;
}

void mmRosControl::cmdVelCallback(const geometry_msgs::TwistPtr& msg) {
  keyCmdVel_ << msg->angular.z, msg->linear.x, msg->linear.y ;
}

void mmRosControl::keyboardCallback(const std_msgs::Bool::ConstPtr& msg ) {
  keyCmdMod_ = msg->data;
  vector_t initTarget(7);
  try {
    listener_->lookupTransform("odom", swerveModelInfo_.eeFrame, ros::Time(0), ee_transform_);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  initTarget.head(3) << ee_transform_.getOrigin().x(), ee_transform_.getOrigin().y(), ee_transform_.getOrigin().z();
  initTarget.tail(4).head(4) << ee_transform_.getRotation().x(), ee_transform_.getRotation().y(), ee_transform_.getRotation().z(),
      ee_transform_.getRotation().w();

  const vector_t zeroInput = vector_t::Zero(swerveModelInfo_.inputDim);
//   TargetTrajectories initTargetTrajectories({observation_.time}, {initTarget}, {zeroInput});
//   rosReferenceManagerPtr_->setTargetTrajectories(std::move(initTargetTrajectories));
}

void mmRosControl::trajectoryCallback(const mm_msg::TargetTrajectories::ConstPtr& msg) {
    std::cout << "Received trajectory callback." << std::endl;
    vector_t state(swerveModelInfo_.stateDim);
 
   {
       std::lock_guard<std::mutex> lock(trajectoryMutex_);
       ObjectTrajectory_.clear();
       state.setZero();
       for (size_t i = 0; i < msg->stateTrajectory.size(); ++i) {
           
           state(x_state_ind) = msg->stateTrajectory[i].data[0]; // qx
           state(y_state_ind) = msg->stateTrajectory[i].data[1]; // q

           double theta_z = msg->stateTrajectory[i].data[2]; // theta_z
           //cal quaternion from theta_z
           state(x_quat_state_ind) = 0.0; // qx
           state(y_quat_state_ind) = 0.0; // qy
           state(z_quat_state_ind) = sin(theta_z / 2.0); // qz
           state(w_quat_state_ind) = cos(theta_z / 2.0); // qw

           state(sh_rot_state_ind) = msg->stateTrajectory[i].data[3]; // q0
           state(sh_rot_state_ind + 1) = msg->stateTrajectory[i].data[4]; // q1
           state(sh_rot_state_ind + 2) = msg->stateTrajectory[i].data[5]; // q2
           state(sh_rot_state_ind + 3) = msg->stateTrajectory[i].data[6]; // q3  
           state(sh_rot_state_ind + 4) = msg->stateTrajectory[i].data[7]; // q4
           state(sh_rot_state_ind + 5) = msg->stateTrajectory[i].data[8]; // q5

           mobile_manipulator::MMState State;
           State.q = getPinocchioJointPosition(state);
           ObjectTrajectory_.push_back(State);
       }

       getTrajectoryFlag_ = true;
   }

}

PLUGINLIB_EXPORT_CLASS(mm_ros_control::mmRosControl, controller_interface::ControllerBase)
} // namespace mm_ros_control