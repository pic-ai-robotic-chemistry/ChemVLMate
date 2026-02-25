#pragma once

#include <atomic>
std::atomic_flag spin_lock = ATOMIC_FLAG_INIT;

#include "selfdefined_trajectory_controller/selfdefined_trajectory_controller.h"

namespace selfdefined_controllers
{
  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      init(HardwareInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    using namespace joint_trajectory_controller;
    using namespace joint_trajectory_controller::internal;

    // ------------0.obtainController Name-----------------
    this->controller_nh_ = controller_nh;
    // Controller Nameselfdefined_trajectory_controller
    this->name_ = getLeafNamespace(this->controller_nh_);
    // Obtain the name of the robotic arm where the controller is located
    this->robot_name_ = getParentNamespace(this->controller_nh_, 1);

    // ------------7.Get defined parameters----------------ur_robot_driveroflaunchCentral Settings
    // wrenchofdead zone,If it is less than the dead zone, set it as0
    controller_nh.param<std::vector<double>>("wrench_dead", wrenchDead_,
                                             std::vector<double>{0.5, 0.5, 0.5, 0.1, 0.1, 0.1});
    // Maximum translation speed
    controller_nh.param<double>("tra_velmax", traVelLimit_, 1.0);
    // Maximum rotational speed
    controller_nh.param<double>("rot_velmax", rotVelLimit_, 1.57);
    // Maximum translational acceleration
    controller_nh.param<double>("tra_accmax", traAccLimit_, 5.0);
    // Maximum rotational acceleration
    controller_nh.param<double>("rot_accmax", rotAccLimit_, 6.28);
    // Whether to print debugging information
    controller_nh.param<bool>("verbose", verbose_, false);

    // ------------8.obtainThe controller controlsofJoint information----------------
    this->joint_names_ = getStrings(this->controller_nh_, "joints");
    if (this->joint_names_.empty())
      return false;
    const unsigned int n_joints = this->joint_names_.size();

    // ------------9.obtainURDFmodel----------------
    std::string robot_description;
    controller_nh.param<std::string>("robot_description", robot_description,
                                     "robot_description");
    urdf::ModelSharedPtr urdf = getUrdf(root_nh, robot_description);
    if (!urdf)
      return false;

    // ------------10.initializationpinocchioDynamics library----------------
    // 10.1 from URDF Model Construction Pinocchio robotmodel-----------
    pinocchio::urdf::buildModel(urdf, this->robot_model_);
    // 10.2 initializationthree pinocchio::Data object-----------
    this->model_curr_data_ = pinocchio::Data(this->robot_model_);    // Current state data of the robot
    this->model_desired_data_ = pinocchio::Data(this->robot_model_); // Expected state data of robots
    this->model_temp_data_ = pinocchio::Data(this->robot_model_);    // Temporary status data of robots
    // 10.3 check compliance_link whether exists in URDF In the model-----------
    std::string compliance_link;
    controller_nh.param<std::string>("compliance_link", compliance_link,
                                     "tool0");
    if (!robot_model_.existFrame(compliance_link))
    {
      const std::string error = "Failed to parse robot chain from urdf model. "
                                "Are you sure that compliance_link exists?";
      ROS_ERROR_STREAM(error);
      throw std::runtime_error(error);
    }

    // 10.4 obtain compliance_link of frame ID-----------
    this->compliance_frameId_ = robot_model_.getFrameId(compliance_link);
    // 10.5 obtain URDF In the modelofJoint information
    std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, this->joint_names_);
    if (urdf_joints.empty())
      return false;
    // 10.6 Claim whether the number of joints matches-----------
    assert(n_joints == urdf_joints.size());
    // 10.7 Initialize member variables-----------
    // 1. initialization Eigen vector，Used to store current and expected joint positions、Speed and acceleration
    this->pino_curr_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_curr_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_acc_ = Eigen::VectorXd::Zero(n_joints);
    // 2. Initialize a 6 line, robot_model_.nv rowof Eigen matrix，Used for storing Jacobian matrices
    this->J_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // Initialize target、currentanderroroftorque（wrench），Their initial values are all zero
    this->target_wrench_.setZero();
    this->current_wrench_.setZero();
    this->error_wrench_.setZero();
    // 3. Translate into English error_wrench_ Write to a real-time buffer（Real-Time Buffer），To ensure data securityofconsistencyandreal-time
    error_wrench_rb_.writeFromNonRT(this->error_wrench_);
    // 4. Adjust this categoryof joints_ and angle_wraparound_ vectorofsize，To adapt to jointsofquantity
    this->joints_.resize(n_joints);
    this->angle_wraparound_.resize(n_joints);

    // 10.8 Initialization loop，Traverse all joints，And executelineCorresponding operations-----------
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      // 1. obtainJoint handle：attemptfromhardware interfaceobtainJoint handle，And stored in joints_ In vectors
      try
      {
        this->joints_[i] = hw->getHandle(this->joint_names_[i]);
        ROS_DEBUG_STREAM_NAMED(this->name_, "Successfully obtained handle for joint '"
                                                << this->joint_names_[i] << "'.");
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(this->name_, "Could not find joint '"
                                                << this->joint_names_[i] << "' in '"
                                                << this->getHardwareInterfaceType() << "'.");
        return false;
      }
      // 2. checkAre the joints continuousof
      this->angle_wraparound_[i] =
          urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
      const std::string not_if = this->angle_wraparound_[i] ? "" : "non-";
      // 3. Record debugging information
      ROS_DEBUG_STREAM_NAMED(this->name_, "Found " << not_if << "continuous joint '"
                                                   << this->joint_names_[i] << "' in '"
                                                   << this->getHardwareInterfaceType() << "'.");
    }
    assert(this->joints_.size() == this->angle_wraparound_.size());
    // 4. Output some debugging information，Including controller name、Number of joints、Hardware interface type and trajectory segment type
    ROS_DEBUG_STREAM_NAMED(this->name_, "Initialized controller '"
                                            << this->name_ << "' with:"
                                            << "\n- Number of joints: " << this->getNumberOfJoints()
                                            << "\n- Hardware interface type: '"
                                            << this->getHardwareInterfaceType() << "'");

    // Subscription targetwrenchand currentwrenchtopic
    std::string topic_target_wrench;
    controller_nh.param<std::string>("topic_target_wrench", topic_target_wrench,
                                     "target_wrench");
    std::string topic_current_wrench;
    controller_nh.param<std::string>("topic_current_wrench", topic_current_wrench,
                                     "current_wrench");
    this->target_wrench_sub_ = controller_nh.subscribe(topic_target_wrench, 1, &SelfDefinedTrajectoryController::targetWrenchCB, this);
    this->current_wrench_sub_ = controller_nh.subscribe(topic_current_wrench, 1, &SelfDefinedTrajectoryController::currentWrenchCB, this);

    // Initialize dynamic configuration server
    this->dyn_conf_server_ = std::make_unique<dynamic_reconfigure::Server<selfdefined_trajectory_controller::selfdefinedControllerConfig>>(controller_nh);
    dynamic_reconfigure::Server<selfdefined_trajectory_controller::selfdefinedControllerConfig>::CallbackType f;
    f = boost::bind(&SelfDefinedTrajectoryController::dynamicReconfigureCallback, this, _1, _2);
    this->dyn_conf_server_->setCallback(f);

    return true;
  }
}