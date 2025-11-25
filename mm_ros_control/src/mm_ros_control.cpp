#include "mm_ros_control/mm_ros_control.hpp"
#include "map_util.hpp"
#include "LoadData.h"
#include <pluginlib/class_list_macros.h>
#include <mm_visualize/mm_config.hpp>
#include <mm_msg/TargetTrajectories.h>
#include <mm_msg/SolverInput.h>
#include <mm_msg/State.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <urdf/model.h>
#include <chrono>
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
    upperBound_ = upperBound;

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
    mmVisConfigPtr_->setUpperBound(upperBound);
    // Initialize tf listener
    listener_.reset(new tf::TransformListener);
    std::string odomTopic = controller_nh.param<std::string>("odom_topic", "/swerve_base/base_pose_ground_truth");
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
    SolverStatePublisher_ = root_nh.advertise<mm_msg::SolverInput>("mobile_manipulator_state", 10);
    StatePublisher_  = root_nh.advertise<mm_msg::State>("mobile_manipulator_current_state", 10);
    TargetStateSubscriber_ = root_nh.subscribe("/swerve_base/mobile_manipulator_target_state", 10, &mmRosControl::targetStateCallback, this);
    eeTargetSubscriber_ = root_nh.subscribe("/swerve_base/task_space_action_server/task_space/goal", 10, &mmRosControl::eeTargetCallback, this);
    
    std::string urdf_param;
    if (!controller_nh.getParam("/ik_robot_description", urdf_param)) {
    ROS_ERROR("Failed to get /ik_robot_description param!");
    } else {
        ROS_INFO_STREAM("Loaded URDF: " << urdf_param.substr(0, 100)); // 打印前100字符
    }
    std::string base_name = "base_link";
    std::string tip_name = "end_effector_link";

    ik_solver_ptr_ = std::make_shared<TRAC_IK::TRAC_IK>(base_name, tip_name, "/ik_robot_description", 0.005, 1e-5, TRAC_IK::Speed);
    
    return true;
}


void mmRosControl::update(const ros::Time& time, const ros::Duration& period) {

    static std::vector<double>  arm_position(6, 0);
    static StateMachine prev_state = IDLE;

    static bool elapsed_time_flag = false;
    {
        
        std::lock_guard<std::mutex> lock(updateOdomMutex_);
        lastObservation_ = currObservation_;
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
        
        // ee_target_ = initTarget;
        solver_thread_ = std::thread(&mmRosControl::SolverThread, this);
        arm_position = std::vector<double>{0.0, -0.5, -1, 0.0, -1.1, 0.0};

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

    mmVisConfigPtr_->UpdatePinocchioModel(getPinocchioJointPosition(currObservation_.state));

    vector_t wheelVels = vector_t::Zero(8);
    vector_t baseVel = vector_t::Zero(3);
    
    if (stateMachine_ != KEYBOARD_CONTROL && stateMachine_ != IDLE){
        auto tmp = currObservation_.state.segment<6>(sh_rot_state_ind).transpose().eval();
        arm_position.assign(tmp.data(), tmp.data() + tmp.size());
    }
    
    switch (stateMachine_)
    {
    case IDLE:
        if(prev_state != IDLE) printf("IDLE mode...\n");
        currInputs.wheelsInput.setZero();
        currInputs.steersInput.setZero();
        currInputs.armInputs.setZero();
        
        for (int i = 0; i < 6; i++)
        {
        currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
        }
        break;

    case TRACKING_TARGET:
    {

        if (prev_state != TRACKING_TARGET) std::cout << "TRACKING_TARGET mode..." << std::endl;
        mm_msg::State stateMsg;
        stateMsg.data[0] = currObservation_.state(x_state_ind);
        stateMsg.data[1] = currObservation_.state(y_state_ind);
        double theta_z = 2 * atan2(currentObservedPose_.orientation.z, currentObservedPose_.orientation.w);
        stateMsg.data[2] = theta_z;
        for (int i = 0; i < 6; i++)
        {
            stateMsg.data[3 + i] = currObservation_.state(sh_rot_state_ind + i);
        }

        StatePublisher_.publish(stateMsg);
        baseVel[0] = 5 * ( C_Target_[2] - theta_z);
        baseVel[1] = 2 * ( C_Target_[0] - currObservation_.state(x_state_ind)) ;
        baseVel[2] = 2 * ( C_Target_[1] - currObservation_.state(y_state_ind)) ;

        if(baseVel[0] < -0.5 )
            baseVel[0] = -0.5;
        else if (baseVel[0] > 0.5)
            baseVel[0] = 0.5;
        if(baseVel[1] < -1 )
            baseVel[1] = -1;
        else if (baseVel[1] > 1)
            baseVel[1] = 1;
        if(baseVel[2] < -1)
            baseVel[2] = -1;
        else if (baseVel[2] > 1)
            baseVel[2] = 1;

        // translated velocity to robot frame
        double cos_theta = cos(theta_z);
        double sin_theta = sin(theta_z);
        double vx = baseVel[1] * cos_theta + baseVel[2] * sin_theta;
        double vy = -baseVel[1] * sin_theta + baseVel[2] * cos_theta;
        baseVel[1] = vx;
        baseVel[2] = vy;

        std::cout << "Base velocity command: "<< baseVel.transpose() << std::endl;
        std::cout << "C_Target_: "<< C_Target_.transpose() << std::endl;
        wheelVels = mmVisConfigPtr_->baseKinemics(baseVel);
        currInputs.wheelsInput = wheelVels.head(4);
        currInputs.steersInput = wheelVels.tail(4);
        for (int i = 0; i < 6; i++)
        {
            currInputs.armInputs(i) =  10 * (C_Target_[3+i] - currObservation_.state(sh_rot_state_ind + i));
        }
       

        break;
    }
    case GET_TRAJECTORY:
    {
        if (prev_state != GET_TRAJECTORY) std::cout << "GET_TRAJECTORY ..." << std::endl;
        // Publish the current state to the topic
        mm_msg::SolverInput stateMsg;
        stateMsg.x0.data[0] = currObservation_.state(x_state_ind);
        stateMsg.x0.data[1] = currObservation_.state(y_state_ind);
        
        double theta_z = 2 * atan2(currentObservedPose_.orientation.z, currentObservedPose_.orientation.w);
        stateMsg.x0.data[2] = theta_z;
        stateMsg.x0.data[3] = currObservation_.state(sh_rot_state_ind);
        stateMsg.x0.data[4] = currObservation_.state(sh_rot_state_ind + 1);
        stateMsg.x0.data[5] = currObservation_.state(sh_rot_state_ind + 2);
        stateMsg.x0.data[6] = currObservation_.state(sh_rot_state_ind + 3);
        stateMsg.x0.data[7] = currObservation_.state(sh_rot_state_ind + 4);
        stateMsg.x0.data[8] = currObservation_.state(sh_rot_state_ind + 5);

        for(int i = 0; i < 9; i++){
            stateMsg.xf.data[i] = traj_target_[i];
        }
        SolverStatePublisher_.publish(stateMsg);
        

        if(ObjectTrajectory_.size() > 0){
            std::lock_guard<std::mutex> lock(trajectoryMutex_);
            mmVisConfigPtr_->DisplayTrajectory(ObjectTrajectory_, 0.5);  
        stateMachine_ = IDLE;
        }

        currInputs.wheelsInput.setZero();
        currInputs.steersInput.setZero();
        currInputs.armInputs.setZero();
        
        for (int i = 0; i < 6; i++)
        {
        currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
        }
        break;
    }
    case EXECUTING_TRAJECTORY:
    {
        static int ObjectPointIndex = 0;
        static auto time =  std::chrono::high_resolution_clock::now();
        static auto start_time =  std::chrono::high_resolution_clock::now();
        if (prev_state != EXECUTING_TRAJECTORY) 
        {
            std::cout << "EXECUTING_TRAJECTORY ..." << std::endl;
            ObjectPointIndex = 0;
            time =  std::chrono::high_resolution_clock::now();
            start_time =  std::chrono::high_resolution_clock::now();
        }

        int objectTrajSize;
        objectTrajSize = ObjectTrajectory_.size();
        
        vector_t startState;
        vector_t ObjectState;
        vector_t Input;
        {
            startState = ObjectTrajectory_[ObjectPointIndex].state;
            ObjectState = ObjectTrajectory_[ObjectPointIndex+1].state;
            Input = ObjectTrajectory_[ObjectPointIndex].input;
            // std::cout << "Current trajectory point ---: " << ObjectPointIndex+1 << "/" << objectTrajSize << std::endl;
        }

        double theta_z = 2 * atan2(currentObservedPose_.orientation.z, currentObservedPose_.orientation.w);
        auto currtime =  std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currtime - time).count();
        // std::cout << "time since last point: "<< elapsed << " milliseconds." << std::endl;
        double alpha = elapsed / 1000.0; // 1.0s between two points
        if (alpha > 1.0) alpha = 1.0;
        vector_t interpState = startState + alpha * (ObjectState - startState);

        baseVel[0] = 5 * (interpState(2) - theta_z);
        baseVel[1] = 10 * ( interpState(0) - currObservation_.state(x_state_ind));
        baseVel[2] = 10 * ( interpState(1) - currObservation_.state(y_state_ind)) ;
        if(baseVel[0] < -0.5 )
            baseVel[0] = -0.5;
        else if (baseVel[0] > 0.5)
            baseVel[0] = 0.5;
        if(baseVel[1] < -1 )
            baseVel[1] = -1;
        else if (baseVel[1] > 1)
            baseVel[1] = 1;
        if(baseVel[2] < -1)
            baseVel[2] = -1;
        else if (baseVel[2] > 1)
            baseVel[2] = 1;

        // translated velocity to robot frame
        double cos_theta = cos(theta_z);
        double sin_theta = sin(theta_z);
        double vx = baseVel[1] * cos_theta + baseVel[2] * sin_theta;
        double vy = -baseVel[1] * sin_theta + baseVel[2] * cos_theta;
        baseVel[1] = vx;
        baseVel[2] = vy;

        wheelVels = mmVisConfigPtr_->baseKinemics(baseVel);
        currInputs.wheelsInput = wheelVels.head(4);
        currInputs.steersInput = wheelVels.tail(4);
        for (int i = 0; i < 6; i++)
        {
            currInputs.armInputs(i) =  10 * (interpState(3+i) - currObservation_.state(sh_rot_state_ind + i));
        }
        
        double dist = (ObjectState.head(2) - currObservation_.state.head(2)).norm() + abs(ObjectState(2) - theta_z);
        if (dist < 0.02 ){
            ObjectPointIndex++;
            time =  std::chrono::high_resolution_clock::now();
            std::cout << "Current trajectory point: " << ObjectPointIndex << "/" << objectTrajSize << std::endl;
            std::cout << "Target position: "<< interpState.head(3).transpose() << std::endl;
            std::cout << "Current position: "<< currObservation_.state.head(3).transpose() << std::endl;
            elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(time - start_time).count();
            std::cout << "time to reach target: "<< elapsed / 1000.0 << " seconds." << std::endl;
            
        }
        


        if (ObjectPointIndex >= objectTrajSize - 1){

            std::cout << "Trajectory execution completed." << std::endl;
            stateMachine_ = IDLE;
            ObjectTrajectory_.clear();
            currInputs.wheelsInput.setZero();
            currInputs.steersInput.setZero();
            currInputs.armInputs.setZero();
        
            for (int i = 0; i < 6; i++)
            {
            currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
            }
            break;
        }
        break;
        
    }
    
    case KEYBOARD_CONTROL:
    {
        if (prev_state != KEYBOARD_CONTROL) std::cout << "KEYBOARD_CONTROL mode..." << std::endl;
        double theta_z = 2 * atan2(currentObservedPose_.orientation.z, currentObservedPose_.orientation.w);
        baseVel<< keyCmdVel_[0], keyCmdVel_[1], keyCmdVel_[2];
        // double cos_theta = cos(theta_z);
        // double sin_theta = sin(theta_z);
        // double vx = baseVel[1] * cos_theta + baseVel[2] * sin_theta;
        // double vy = -baseVel[1] * sin_theta + baseVel[2] * cos_theta;
        // baseVel[1] = vx;
        // baseVel[2] = vy;
    
        wheelVels = mmVisConfigPtr_->baseKinemics(baseVel);

        currInputs.wheelsInput = wheelVels.head(4);
        currInputs.steersInput = wheelVels.tail(4);
        for (int i = 0; i < 6; i++)
        {
        currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
        }
        break;
    }

    default:
        break;
    }
    prev_state = stateMachine_;
    currInputs = upperBoundConstraints(currInputs, upperBound_);
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
mmRosControl::RobotInputs mmRosControl::upperBoundConstraints(mmRosControl::RobotInputs inputs, vector_t upperBound){
    for (int i = 0; i < 4; i++){
        if (inputs.wheelsInput(i) > upperBound( lb_wheel_input_ind + 2*i)){
            inputs.wheelsInput(i) = upperBound( lb_wheel_input_ind + 2*i);
        }
        else if (inputs.wheelsInput(i) < -upperBound( lb_wheel_input_ind + 2*i)){
            inputs.wheelsInput(i) = -upperBound( lb_wheel_input_ind + 2*i);
        }

        if (inputs.steersInput(i) > upperBound( lb_steer_input_ind + 2*i)){
            inputs.steersInput(i) = upperBound( lb_steer_input_ind + 2*i);
        }
        else if (inputs.steersInput(i) < -upperBound( lb_steer_input_ind + 2*i)){
            inputs.steersInput(i) = -upperBound( lb_steer_input_ind + 2*i);
        }
    }

    for (int i = 0; i < 6; i++){
        if (inputs.armInputs(i) > upperBound( sh_rot_input_ind + i)){
            inputs.armInputs(i) = upperBound( sh_rot_input_ind + i);
        }
        else if (inputs.armInputs(i) < -upperBound( sh_rot_input_ind + i)){
            inputs.armInputs(i) = -upperBound( sh_rot_input_ind + i);
        }
    }
    return inputs;
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
    if (keyCmdMod_) {
        stateMachine_ = KEYBOARD_CONTROL;
    }
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
            State.state = vector_t::Zero(9);
            State.input = vector_t::Zero(9);
            for (int j = 0; j < 9; j++){
                State.state(j) = msg->stateTrajectory[i].data[j];
                State.input(j) = msg->inputTrajectory[i].data[j];
            }
            std::cout << "Trajectory point " << i << ": " << State.state.transpose() << std::endl;
              
           ObjectTrajectory_.push_back(State);
       }

       stateMachine_ = GET_TRAJECTORY;
   }

}

void mmRosControl::targetStateCallback(const mm_msg::State::ConstPtr& msg) {
    vector_t target(9);
    for(id_t i = 0; i < 9; i++){
        target(i) = msg->data[i];
    }
    C_Target_ = target;
    stateMachine_ = TRACKING_TARGET;
}

void mmRosControl::eeTargetCallback(const manipulation_msgs::ReachPoseActionGoal::ConstPtr& msg) {
    vector_t target(7);
    target(0) = msg->goal.poseStamped.pose.position.x;
    target(1) = msg->goal.poseStamped.pose.position.y;
    target(2) = msg->goal.poseStamped.pose.position.z;
    target(3) = msg->goal.poseStamped.pose.orientation.x;
    target(4) = msg->goal.poseStamped.pose.orientation.y;
    target(5) = msg->goal.poseStamped.pose.orientation.z;
    target(6) = msg->goal.poseStamped.pose.orientation.w;
    std::cout << "Received ee target: " << target.transpose() << std::endl;
    KDL::Chain chain;
    KDL::JntArray lower_limits, upper_limits;
    bool valid = ik_solver_ptr_->getKDLChain(chain);
    if (!valid) {
        std::cout << "Failed to get KDL chain from IK solver!" << std::endl;
        return;
    }
    
    valid = ik_solver_ptr_->getKDLLimits(lower_limits, upper_limits);
    if (!valid) {
        std::cout << "Failed to get KDL joint limits from IK solver!" << std::endl;
        return;
    }

    KDL::JntArray nominal(chain.getNrOfJoints());

    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
        {nominal(i) = (lower_limits(i) + upper_limits(i)) / 2.0;
            nominal(i) = (lower_limits(i) + upper_limits(i)) / 2.0;
            std::cout << nominal(i) << " ";
        }
    std::cout << std::endl;
    

    KDL::Frame  target_pose(KDL::Vector(target(0), target(1), target(2)));
    KDL::Rotation rot = KDL::Rotation::Quaternion(target(6), target(3), target(4), target(5));
    KDL::JntArray result(chain.getNrOfJoints());    
    int rc = ik_solver_ptr_->CartToJnt(nominal, target_pose, result);
    traj_target_ = vector_t::Zero(9);   
    if (rc >= 0) {
        std::cout << "IK result: ";
        for (unsigned int i = 0; i < result.rows(); i++)
        {
            std::cout << result(i) << " ";
            traj_target_[i] = result(i);
        }
        std::cout << std::endl;

        

        stateMachine_ = GET_TRAJECTORY;
    } else {
        std::cout << "IK solver failed with error code: " << rc << std::endl;
    }
}

PLUGINLIB_EXPORT_CLASS(mm_ros_control::mmRosControl, controller_interface::ControllerBase)
} // namespace mm_ros_control