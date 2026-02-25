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
    // obtainThe controller is locatedofRobot arm name
    this->robot_name_ = getParentNamespace(this->controller_nh_, 1);

    // ------------1.Set the update frequency for controller releases----------------ur_robot_driveroflaunchChineseSettings300
    double state_publish_rate = 50.0;
    this->controller_nh_.getParam("state_publish_rate", state_publish_rate);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Controller state will be published at "
                                            << state_publish_rate << "Hz.");
    // Set publishing frequency50Hz
    this->state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

    // ------------2.Set the update frequency for action monitoring------------------ur_robot_driveroflaunchChineseSettings20
    double action_monitor_rate = 20.0;
    this->controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Action status changes will be monitored at "
                                            << action_monitor_rate << "Hz.");
    // Set monitoring frequency20Hz
    this->action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);

    // ------------3.Set the duration of stopping the trajectory----------------inur_robot_driveroflaunchChineseSettings0.5
    double stop_trajectory_duration = 0.0;
    this->controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration);
    ROS_DEBUG_STREAM_NAMED(this->name_, "Stop trajectory has a duration of "
                                            << stop_trajectory_duration << "s.");
    this->stop_trajectory_duration_ = stop_trajectory_duration;

    // ------------4.check whetherallow_partial_joints_goal_Settings----------------eyefrontSettingsforfalse
    controller_nh.param<bool>("allow_partial_joints_goal",
                              this->allow_partial_joints_goal_, false);
    // If partial joint targets are allowed，Output debugging information
    if (this->allow_partial_joints_goal_)
    {
      ROS_DEBUG_NAMED(this->name_, "Goals with partial set of joints are allowed");
    }

    // ------------5.obtaineyemarktorque/whenfrontName of torque topic----------------
    // subscribed target wrench topic name
    std::string topic_target_wrench;
    controller_nh.param<std::string>("topic_target_wrench", topic_target_wrench,
                                     "target_wrench");
    ROS_DEBUG_STREAM_NAMED(this->name_, "Controller target wrench will be subscribed at "
                                            << topic_target_wrench);
    // subscribed current wrench topic name
    std::string topic_current_wrench;
    controller_nh.param<std::string>("topic_current_wrench", topic_current_wrench,
                                     "current_wrench");
    ROS_INFO_STREAM_NAMED(this->name_, "Controller current wrench will be subscribed at "
                                           << topic_current_wrench);

    std::string topic_currel_errorpose;
    controller_nh.param<std::string>("topic_currel_errorpose", topic_currel_errorpose,
                                     "error_pos_otherToself");
    ROS_INFO_STREAM_NAMED(this->name_, "Controller current pose relerror will be subscribed at "
                                           << topic_currel_errorpose);

    // ------------6.Obtain the smooth and compliant joints that are affected----------------right_tool0orleft_tool0
    controller_nh.param<std::string>("compliance_link", this->compliance_link_,
                                     "tool0");
    // ------------7.Get defined parameters----------------ur_robot_driveroflaunchChineseSettings
    // wrenchThe Dead Zone,If it is less than the dead zone, set it as0
    controller_nh.param<std::vector<double>>("wrench_dead", wrenchDead_,
                                             std::vector<double>{0.5, 0.5, 0.5, 0.1, 0.1, 0.1});
    // maximumtranslationspeed
    controller_nh.param<double>("tra_velmax", traVelLimit_, 1.0);
    // Maximum rotationspeed
    controller_nh.param<double>("rot_velmax", rotVelLimit_, 1.57);
    // maximumTranslational acceleration
    controller_nh.param<double>("tra_accmax", traAccLimit_, 5.0);
    // Maximum rotationplusspeed
    controller_nh.param<double>("rot_accmax", rotAccLimit_, 6.28);
    // Whether to print debugging information
    controller_nh.param<bool>("verbose", verbose_, false);

    // ------------8.Obtain the joint information controlled by the controller----------------
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
    // 10.1 from URDF modelbuild Pinocchio robotmodel-----------
    pinocchio::urdf::buildModel(urdf, this->robot_model_);
    // 10.2 initializationthreea pinocchio::Data object-----------
    this->model_curr_data_ = pinocchio::Data(this->robot_model_);    // Current state data of the robot
    this->model_desired_data_ = pinocchio::Data(this->robot_model_); // robotexpectationstatusdata
    this->model_temp_data_ = pinocchio::Data(this->robot_model_);    // Temporary status data of robots
    // 10.3 check compliance_link Whether to saveinYu URDF In the model-----------
    if (!robot_model_.existFrame(compliance_link_))
    {
      const std::string error = "Failed to parse robot chain from urdf model. "
                                "Are you sure that compliance_link exists?";
      ROS_ERROR_STREAM(error);
      throw std::runtime_error(error);
    }

    // 10.4 obtain compliance_link of frame ID-----------
    this->compliance_frameId_ = robot_model_.getFrameId(compliance_link_);
    // 10.5 obtain URDF In the modelofJoint information
    std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, this->joint_names_);
    if (urdf_joints.empty())
      return false;
    // 10.6 Claim whether the number of joints matches-----------
    assert(n_joints == urdf_joints.size());
    // 10.7 initializationmember variable-----------
    // 1. initialization Eigen vector，Used to store current and expected joint positions、speedandplusspeed
    this->pino_curr_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_curr_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_pos_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_vel_ = Eigen::VectorXd::Zero(n_joints);
    this->pino_desired_acc_ = Eigen::VectorXd::Zero(n_joints);
    // 2. initializationonea 6 line, robot_model_.nv Column of Eigen matrix，Used for storing Jacobian matrices
    this->J_ = Eigen::MatrixXd::Zero(6, robot_model_.nv);
    // initializationeyemark、whenfrontanderroroftorque（wrench），Their initial values are all zero
    this->target_wrench_.setZero();
    this->current_wrench_.setZero();
    this->error_wrench_.setZero();
    /////////////////////////////
    for (size_t i = 0; i < 6; i++)
      for(size_t j = 0; j < 3; j++)
          error_zforce[i][j] = 0.0;
    /////////////////////////////
    // 3. Translate into English error_wrench_ writeoneaReal time buffer（Real-Time Buffer），To ensure data consistency and real-time performance
    error_wrench_rb_.writeFromNonRT(this->error_wrench_);
    // 4. Adjust this category joints_ and angle_wraparound_ vectorofsize，To accommodate the number of joints
    this->joints_.resize(n_joints);
    this->angle_wraparound_.resize(n_joints);

    // 10.8 initializationcycle，traverseall joints，And perform the corresponding operations-----------
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      // 1. obtainjointhandle：attemptfromhardware interfaceobtainjointhandle，And stored in joints_ vectorChinese
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
      // 2. Check if the joints are continuous
      this->angle_wraparound_[i] =
          urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
      const std::string not_if = this->angle_wraparound_[i] ? "" : "non-";
      // 3. Record debugging information
      ROS_DEBUG_STREAM_NAMED(this->name_, "Found " << not_if << "continuous joint '"
                                                   << this->joint_names_[i] << "' in '"
                                                   << this->getHardwareInterfaceType() << "'.");
    }
    assert(this->joints_.size() == this->angle_wraparound_.size());
    // 4. Output some debugging information，includingController Name、Number of joints、hardware interfacetypeandtrajectoryparagraphtype
    ROS_DEBUG_STREAM_NAMED(this->name_, "Initialized controller '"
                                            << this->name_ << "' with:"
                                            << "\n- Number of joints: " << this->getNumberOfJoints()
                                            << "\n- Hardware interface type: '"
                                            << this->getHardwareInterfaceType() << "'"
                                            << "\n- Trajectory segment type: '"
                                            << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

    // ------------11.Set tolerance----------------
    ros::NodeHandle tol_nh(this->controller_nh_, "constraints");
    this->default_tolerances_ = getSegmentTolerances<typename Base::Scalar>(tol_nh, this->joint_names_);

    // ------------12.initializationHardware interface adapter----------------
    this->hw_iface_adapter_.init(this->joints_, this->controller_nh_);

    // ------------13.Create topic subscription----------------
    // 1. Reference trajectory subscription：/selfdefined_trajectory_controller/command
    this->trajectory_command_sub_ = this->controller_nh_.subscribe(
        "command", 1, &SelfDefinedTrajectoryController::trajectoryCommandCB, this);
    // ----- Dual arm reference trajectory topic
    this->dual_trajectory_command_sub_ = this->controller_nh_.subscribe(
        "/dual_robot/selfdefined_trajectory_controller/command", 1,
        &SelfDefinedTrajectoryController::dualtrajectoryCommandCB, this);
    // 2. subscribe whenfrontwrench oftopic
    this->current_wrench_sub_ = this->controller_nh_.subscribe(topic_current_wrench, 1,
                                                               &SelfDefinedTrajectoryController::currentWrenchCB, this,
                                                               ros::TransportHints().reliable().tcpNoDelay()); // Transmission prompt，Set transmission to reliable and disable latency
    // 3. subscribe expectationwrench oftopic
    this->target_wrench_sub_ = this->controller_nh_.subscribe(topic_target_wrench, 1,
                                                              &SelfDefinedTrajectoryController::targetWrenchCB, this);

    // 4. subscribe whenfronterror_RtoLoftopic
    this->currel_errorpose_sub_ = this->controller_nh_.subscribe(topic_currel_errorpose, 1,
                                                                 &SelfDefinedTrajectoryController::currentRelerrorposCB, this,
                                                                 ros::TransportHints().reliable().tcpNoDelay()); // Transmission prompt，Set transmission to reliable and disable latency

    // ------------14.Create a topic to publish----------------
    // 1. Create a new one StatePublisher object，Used to publish status information of controllers
    this->state_publisher_.reset(new StatePublisher(this->controller_nh_, "state", 1));
    // 2. Create a new one ActionServer object，Used for handling action targets and cancellation requests
    this->action_server_.reset(new ActionServer(this->controller_nh_, "follow_joint_trajectory",
                                                std::bind(&SelfDefinedTrajectoryController::goalCB, this, std::placeholders::_1),
                                                std::bind(&SelfDefinedTrajectoryController::cancelCB, this, std::placeholders::_1),
                                                false));
    // 3. Start actionservicedevice，Start processing action requests
    this->action_server_->start();
    // 4. Create and advertise service query_state，Used to query the status of the controller
    this->query_state_service_ = this->controller_nh_.advertiseService(
        "query_state", &SelfDefinedTrajectoryController::queryStateService, this);

    // ------------15.Pre allocated resources----------------
    // 1. Store data in different states
    this->current_state_ = typename Segment::State(n_joints);
    this->old_desired_state_ = typename Segment::State(n_joints);
    this->desired_state_ = typename Segment::State(n_joints);
    this->state_error_ = typename Segment::State(n_joints);
    this->desired_joint_state_ = typename Segment::State(1);
    this->state_joint_error_ = typename Segment::State(1);
    // 2. Successful joint trajectory
    this->successful_joint_traj_ = boost::dynamic_bitset<>(this->getNumberOfJoints());
    // 3. createandcheckmaintaintrajectory
    this->hold_trajectory_ptr_ = this->createHoldTrajectory(n_joints);
    assert(this->joint_names_.size() == this->hold_trajectory_ptr_->size());

    // ------------16.Choose different options TrajectoryBuilder type----------------
    if (this->stop_trajectory_duration_ == 0.0)
    { // controldeviceWill generateoneamaintainwhenfrontstatusoftrajectory
      this->hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl>>(
          new HoldTrajectoryBuilder<SegmentImpl, HardwareInterface>(this->joints_));
    }
    else
    { // The controller will generate a stopped trajectory, thattrajectoryTranslate into Englishin stop_trajectory_duration_ insideTranslate into EnglishjointstatusSlow down to stop
      this->hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl>>(
          new StopTrajectoryBuilder<SegmentImpl>(this->stop_trajectory_duration_, this->desired_state_));
    }

    // ------------17.Settingsdynamic configurationservicedevice,Used for dynamic adjustmentAdmittance controllerChineseofM,D,Kthreeamatrixofparameter------------
    dyn_conf_server_.reset(new dynamic_reconfigure::Server<
                           selfdefined_trajectory_controller::selfdefinedControllerConfig>(this->controller_nh_));
    dyn_conf_server_->setCallback(
        std::bind(&SelfDefinedTrajectoryController::dynamicReconfigureCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    // --------------------------------------------------------------------------------------

    // ------------18.initialization了statusmessageofeachafield，andinComplete modificationsafterUnlock publisher----------------
    {
      this->state_publisher_->lock();
      this->state_publisher_->msg_.joint_names = this->joint_names_;
      this->state_publisher_->msg_.desired.positions.resize(n_joints);
      this->state_publisher_->msg_.desired.velocities.resize(n_joints);
      this->state_publisher_->msg_.desired.accelerations.resize(n_joints);
      this->state_publisher_->msg_.actual.positions.resize(n_joints);
      this->state_publisher_->msg_.actual.velocities.resize(n_joints);
      this->state_publisher_->msg_.error.positions.resize(n_joints);
      this->state_publisher_->msg_.error.velocities.resize(n_joints);
      this->state_publisher_->unlock();
    }
    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      update(const ros::Time &time, const ros::Duration &period)
  {
    // ------------1.Get the current trajectory that needs to be executed----------------
    // typenameKeywords are used to specifyBaseclassofmembertypeTrajectoryPtr，To ensure that the compiler correctly parses the type
    typename Base::TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    typename Base::Trajectory &curr_traj = *curr_traj_ptr;

    // ------------2.Update time data----------------（so thatincontroldevicetransportlineduringmaintainmostnewofTime informationandControl cycle）
    typename Base::TimeData time_data;
    time_data.time = time;                                                       // Cache the current time
    time_data.period = ros::Duration(period.toSec());                            // Cache the current control cycle2ms
    time_data.uptime = this->time_data_.readFromRT()->uptime + time_data.period; // morenewcontroldevicetransportlinetime
    this->time_data_.writeFromNonRT(time_data);                                  // Write new time data

    // ------------3.morenewwhenfrontstatusandstatuserror---------------
    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      // 3.1 Useuponeperiodofcontroljointstatusmake, do, create, compose, write, etc.forwhenfrontjointstatus（position，speed，(plusspeed)）(Terminal Cartesian space)
      this->current_state_.position[i] = this->desired_state_.position[i];
      this->pino_curr_pos_(i) = this->desired_state_.position[i];
      this->current_state_.velocity[i] = this->desired_state_.velocity[i];
      this->pino_curr_vel_(i) = this->desired_state_.velocity[i];
      // 3.2 obtainitemiExpected state of a joint（position，speed，plusspeed）(Joint space)
      // Assign todesired_joint_state_, typename Segment::State(1);
      typename Base::TrajectoryPerJoint::const_iterator segment_it = sample(
          curr_traj[i], time_data.uptime.toSec(), this->desired_joint_state_);
      if (curr_traj[i].end() == segment_it)
      {
        // If no trajectory is defined at the current time，Record error logs
        ROS_ERROR_NAMED(this->name_,
                        "Unexpected error: No trajectory defined at current "
                        "time. Please contact the package maintainer.");
        return;
      }
      // 3.4 obtainwhenfrontmomentoftheoryjointstatuspino_desired_
      this->pino_desired_pos_(i) = this->desired_joint_state_.position[0];
      this->pino_desired_vel_(i) = this->desired_joint_state_.velocity[0];
      this->pino_desired_acc_(i) = this->desired_joint_state_.acceleration[0];
      // 3.5 compareexpectationjointanduponeperiodofcontroljointstatusofdifference
      // Position error passed throughshortest_angular_distanceTo calculate，To ensure the angleerrorin[-π, π]within the scope of
      this->state_error_.position[i] = angles::shortest_angular_distance(
          this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_error_.velocity[i] =
          this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_error_.acceleration[i] = 0.0;
      // 3.6 Check if the trajectory execution time has been reached
      const typename Base::RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
      if (rt_segment_goal && rt_segment_goal == this->rt_active_goal_)
      {
        if (segment_it == --curr_traj[i].end())
        {
          const ros::Time uptime = this->time_data_.readFromRT()->uptime;
          if (uptime.toSec() < segment_it->endTime())
            ;
          else
            this->successful_joint_traj_[i] = 1;
        }
      }
    }

    // ------------4.By updatingpinocchioobtainRobots asfrontstatus---------------
    // 4.1 Update the current status of the robot
    pinocchio::forwardKinematics(robot_model_, model_curr_data_,
                                 pino_curr_pos_, pino_curr_vel_);
    pinocchio::updateFramePlacements(robot_model_, model_curr_data_);
    pino_rstate_rb_.writeFromNonRT(model_curr_data_);
    // 4.2 obtaincompliance_link_The actual location（relative tobozi_link？)
    curr_pose_ = model_curr_data_.oMf[compliance_frameId_]; // pinocchio::SE3
    // 4.3 obtaincompliance_link_ofactualendspeed
    curr_vel_ = pinocchio::getFrameVelocity(robot_model_, model_curr_data_,
                                            compliance_frameId_,
                                            pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion

    // ------------5.By updatingpinocchioobtainrobotexpectationstatus---------------
    // 5.1 morenewrobotexpectationstatus
    pinocchio::forwardKinematics(robot_model_, model_desired_data_,
                                 pino_desired_pos_, pino_desired_vel_,
                                 pino_desired_acc_);
    // 5.2 obtaincompliance_link_Expected location，relative tobase_link)
    desired_pose_ = pinocchio::updateFramePlacement(robot_model_,
                                                    model_desired_data_,
                                                    compliance_frameId_); // pinocchio::SE3
    // 5.3 obtaincompliance_link_ofexpectationendspeed
    desired_vel_ = pinocchio::getFrameVelocity(robot_model_, model_desired_data_,
                                               compliance_frameId_,
                                               pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion
    // 5.3 obtaincompliance_link_ofexpectationEnd addspeed
    desired_acc_ = pinocchio::getFrameAcceleration(robot_model_, model_desired_data_,
                                                   compliance_frameId_,
                                                   pinocchio::LOCAL_WORLD_ALIGNED); // pinocchio::Motion

    ////////////////////////////////Controller algorithm//////////////////////////////////////////
    admittance_control(period);
    //////////////////////////////////////////////////////////////////////////////////

    // ------------10.Printing the relevant parameters of the controller--------------
    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(1, "curr_q: " << std::endl
                                             << pino_curr_pos_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "desired_q:" << std::endl
                                               << pino_desired_pos_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "curr_qv: " << std::endl
                                              << pino_curr_vel_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "desired_qv:" << std::endl
                                                << pino_desired_vel_.transpose());
      ROS_INFO_STREAM_THROTTLE(1, "x_e_:" << std::endl
                                          << x_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "dx_e_:" << std::endl
                                           << dx_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "ddx_e_:" << std::endl
                                            << ddx_e_.toVector().transpose() << std::endl);
      ROS_INFO_STREAM_THROTTLE(1, "error_wrench:" << std::endl
                                                  << " f= " << error_wrench_.topRows(3).transpose() << std::endl
                                                  << " t = " << error_wrench_.bottomRows(3).transpose());
    }

    // ------------11.checkwhenfrontIs there an activity objective--------------
    typename Base::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    if (current_active_goal &&
        this->successful_joint_traj_.count() == this->joints_.size())
    {
      current_active_goal->preallocated_result_->error_code =
          control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
      // Clean up related resources，Ensure that feedback information is not released again，And reset the relevant variables
      current_active_goal.reset(); // do not publish feedback
      this->rt_active_goal_.reset();
      this->successful_joint_traj_.reset();
    }

    // ------------12.Hardware interface adapter：Generate and send commands--------------
    this->hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                          this->desired_state_,
                                          this->state_error_);

    // ------------13.helpupLayer systemorUser communitynoodlesReal time monitoring controllerofstatus--------------
    // checkwhenfrontIs there an activity objective
    if (current_active_goal)
    {
      // Set the timestamp for feedback information
      current_active_goal->preallocated_feedback_->header.stamp = this->time_data_.readFromRT()->time;
      // Settingsexpectationstatusoffeedback information
      current_active_goal->preallocated_feedback_->desired.positions = this->desired_state_.position;
      current_active_goal->preallocated_feedback_->desired.velocities = this->desired_state_.velocity;
      current_active_goal->preallocated_feedback_->desired.accelerations = this->desired_state_.acceleration;
      // Settingsactualstatusoffeedback information
      current_active_goal->preallocated_feedback_->actual.positions = this->current_state_.position;
      current_active_goal->preallocated_feedback_->actual.velocities = this->current_state_.velocity;
      // Settingsstatuserroroffeedback information
      current_active_goal->preallocated_feedback_->error.positions = this->state_error_.position;
      current_active_goal->preallocated_feedback_->error.velocities = this->state_error_.velocity;
      // Send feedback information
      current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
    }

    // ------------14.Publish the current time status of the system--------------
    this->publishState(time_data.uptime);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      currentWrenchCB(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    /////////////////------Single arm independent mode------//////////////////////////
    // ------------1.dead zoneSettingsExclude disturbance terms andAssign tocurrent_wrench_--------------
    current_wrench_.linear()(0) =
        abs(msg->wrench.force.x) < wrenchDead_[0] ? 0 : msg->wrench.force.x;
    current_wrench_.linear()(1) =
        abs(msg->wrench.force.y) < wrenchDead_[1] ? 0 : msg->wrench.force.y;
    current_wrench_.linear()(2) =
        abs(msg->wrench.force.z) < wrenchDead_[2] ? 0 : msg->wrench.force.z;
    current_wrench_.angular()(0) =
        abs(msg->wrench.torque.x) < wrenchDead_[3] ? 0 : msg->wrench.torque.x;
    current_wrench_.angular()(1) =
        abs(msg->wrench.torque.y) < wrenchDead_[4] ? 0 : msg->wrench.torque.y;
    current_wrench_.angular()(2) =
        abs(msg->wrench.torque.z) < wrenchDead_[5] ? 0 : msg->wrench.torque.z;

    // Force position mixing——Grinding operation
    if (this->robot_name_ == "right_robot")
    {
      // Grinding with the right hand，only focus onzAxis expansion and contraction
      current_wrench_.linear()(0) = 0;
      current_wrench_.linear()(1) = 0;
      current_wrench_.angular()(0) = 0;
      current_wrench_.angular()(1) = 0;
      current_wrench_.angular()(2) = 0;
    }
    else if(this->robot_name_ == "left_robot")
    {
      // Left hand assistance，Guan Zhupingnoodlesmortaroftranslation，eyefrontplusHold it. Yesxoznoodles
      // current_wrench_.angular()(0) = 0;
      // current_wrench_.angular()(1) = 0;
      // current_wrench_.angular()(2) = 0;
    }
    /////////////////------Single arm independent mode------//////////////////////////

    // /////////////////------Dual arm collaboration mode------//////////////////////////
    // // Force position mixing——Grinding operation(This mode，The torque information has been replaced withworldUnder the coordinate system，And the torque isright_wrench)
    // if (this->robot_name_ == "right_robot")
    // {
    //   // Grinding with the right hand，only focus onzAxis expansion and contraction
    //   current_wrench_.linear()(0) = (1-Coor_para/100.0) * msg->wrench.force.x;
    //   current_wrench_.linear()(1) = (1-Coor_para/100.0) * msg->wrench.force.y;
    //   current_wrench_.linear()(2) = (1-0) * msg->wrench.force.z;
    //   current_wrench_.angular()(0) = msg->wrench.torque.x;
    //   current_wrench_.angular()(1) = msg->wrench.torque.y;
    //   current_wrench_.angular()(2) = msg->wrench.torque.z;
    // }
    // else if(this->robot_name_ == "left_robot")
    // {
    //   // Left hand assistance，Guan Zhupingnoodlesmortaroftranslation，eyefrontplusHold it. Yesxoznoodles
    //   current_wrench_.linear()(0) = (-1*Coor_para/100.0) * msg->wrench.force.x;
    //   current_wrench_.linear()(1) = (-1*Coor_para/100.0) * msg->wrench.force.y;
    //   current_wrench_.linear()(2) = (1-1) * msg->wrench.force.z;
    //   current_wrench_.angular()(0) = msg->wrench.torque.x;
    //   current_wrench_.angular()(1) = msg->wrench.torque.y;
    //   current_wrench_.angular()(2) = msg->wrench.torque.z;
    // }
    // /////////////////------Dual arm collaboration mode------//////////////////////////

    // ------------2.fromReal time data buffer reading for robotic armfrontstatusinformation--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // ------------3.Calculate torque error--------------
    // 3.1 transformationwrenchThe reference coordinate system isbase_link
    // current_wrench_inframe_idup
    // model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].actTranslate into Englishcurrent_wrench_Convert tobase_linkChinese
    current_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(current_wrench_);
    // std::cout << "current_wrench: " << current_wrench_ << std::endl;
    // 3.2 Using spin locks to protect shared resources
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.3 Calculate torque error，And save itinerror_wrench_rb_Chinese，convenientupdatefunctionChineseinvoke
    error_wrench_rb_.writeFromNonRT(current_wrench_ - target_wrench_);
    // 3.4 Release spin lock
    spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      currentRelerrorposCB(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    //////////////////////////////////////////////////////
    tf2::Quaternion quaternion(msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w);
    double roll, pitch, yaw;
    // Euler anglesX(roll)   Y(pitch)    Z(yaw)
    tf2::Matrix3x3(quaternion).getEulerYPR(yaw, pitch, roll);

    current_wrench_.linear()(0) = -1 * msg->pose.position.x*500;
    current_wrench_.linear()(1) = -1 * msg->pose.position.y*500;
    current_wrench_.linear()(2) = -1 * msg->pose.position.z*500;
    current_wrench_.angular()(0) = -1 * roll * 500;
    current_wrench_.angular()(1) = -1 * pitch * 500;
    current_wrench_.angular()(2) = -1 * yaw * 500;

    // ------------2.fromReal time data buffer reading for robotic armfrontstatusinformation--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // ------------3.Calculate torque error--------------
    // 3.1 transformationwrenchThe reference coordinate system isbase_link
    // current_wrench_inframe_idup
    // model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].actTranslate into Englishcurrent_wrench_Convert tobase_linkChinese
    current_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(current_wrench_);
    // std::cout << "current_wrench: " << current_wrench_ << std::endl;
    // 3.2 Using spin locks to protect shared resources
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.3 Calculate torque error，And save itinerror_wrench_rb_Chinese，convenientupdatefunctionChineseinvoke
    error_wrench_rb_.writeFromNonRT(current_wrench_);
    // 3.4 Release spin lock
    spin_lock.clear(std::memory_order_release);

    //////////////////////////////////////////////////////

    // // Extract the pose of the target frame (geometry_msgs::PoseStamped -> pinocchio::SE3)
    // Eigen::Vector3d position(msg->pose.position.x,
    //                          msg->pose.position.y,
    //                          msg->pose.position.z);

    // Eigen::Quaterniond quaternion(msg->pose.orientation.w,
    //                               msg->pose.orientation.x,
    //                               msg->pose.orientation.y,
    //                               msg->pose.orientation.z);

    // currel_errorpose_ = pinocchio::SE3(quaternion.toRotationMatrix(), position);

    // // ------------2.fromReal time data buffer reading for robotic armfrontstatusinformation--------------
    // model_temp_data_ = *(pino_rstate_rb_.readFromRT());

    // // ------------3.Calculate relative pose error--------------
    // // 3.1 transformationwrenchThe reference coordinate system isbase_link
    // // currel_errorpose_inframe_idup
    // // model_temp_data_.oMf[msg->header.frame_id] *Translate into Englishcurrent_wrench_Convert toworldChinese
    // currel_errorpose_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)] * currel_errorpose_;
    // // std::cout << "currel_errorpose_: " << currel_errorpose_ << std::endl;
    // // 3.2 Using spin locks to protect shared resources
    // while (spin_lock.test_and_set(std::memory_order_acquire))
    //   ;
    // // 3.3 Calculate torque error，And save itincurrel_errorpose_rb_Chinese，convenientupdatefunctionChineseinvoke
    // currel_errorpose_rb_.writeFromNonRT(currel_errorpose_);
    // // 3.4 Release spin lock
    // spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      targetWrenchCB(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    // ------------1.Assign totarget_wrench_--------------
    target_wrench_.linear()(0) = msg->wrench.force.x;
    target_wrench_.linear()(1) = msg->wrench.force.y;
    target_wrench_.linear()(2) = msg->wrench.force.z;
    target_wrench_.angular()(0) = msg->wrench.torque.x;
    target_wrench_.angular()(1) = msg->wrench.torque.y;
    target_wrench_.angular()(2) = msg->wrench.torque.z;

    // target_wrench_.linear()(0) = 0;
    // target_wrench_.linear()(1) = 0;
    // target_wrench_.linear()(2) = msg->wrench.force.z;
    // target_wrench_.angular()(0) = 0;
    // target_wrench_.angular()(1) = 0;
    // target_wrench_.angular()(2) = 0;

    // ------------2.fromReal time data buffer reading for robotic armfrontstatusinformation--------------
    model_temp_data_ = *(pino_rstate_rb_.readFromRT());
    // ------------3.Receive reference torque--------------
    // 3.1 Using spin locks to protect shared resources
    while (spin_lock.test_and_set(std::memory_order_acquire))
      ;
    // 3.2 transformationwrenchThe reference coordinate system isright/left_base_link
    target_wrench_ = model_temp_data_.oMf[robot_model_.getFrameId(msg->header.frame_id)].act(target_wrench_);
    // std::cout << "target_wrench: " << target_wrench_ << std::endl;
    // 3.3 Release spin lock
    spin_lock.clear(std::memory_order_release);
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      dynamicReconfigureCallback(selfdefined_trajectory_controller::selfdefinedControllerConfig
                                     &config,
                                 uint32_t level)
  {
    Vector6D tmp_M, tmp_D, tmp_K;
    for (size_t i = 0; i < 3; i++)
    {
      tmp_M[i] = config.F_M_para;
      tmp_D[i] = config.F_D_para;
      tmp_K[i] = config.F_K_para;
      tmp_M[i + 3] = config.T_M_para;
      tmp_D[i + 3] = config.T_D_para;
      tmp_K[i + 3] = config.T_K_para;
    }
    this->AD_kp = config.AD_kp;
    this->AD_ki = config.AD_ki;
    this->AD_kd = config.AD_kd;
    this->M_ = tmp_M.asDiagonal();
    this->D_ = tmp_D.asDiagonal();
    this->K_ = tmp_K.asDiagonal();
    this->Coor_para = config.Coor_para;
  }

  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      updateTrajectoryCommand(const JointTrajectoryConstPtr &msg,
                              RealtimeGoalHandlePtr gh,
                              std::string *error_string)
  {
    typedef joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> Options;
    Options options;
    options.error_string = error_string;
    std::string error_string_tmp;

    // ------------1.Pre inspection conditions：Check if the controller is running--------------
    if (!this->isRunning())
    {
      error_string_tmp = "Can't accept new commands. Controller is not running.";
      ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }
    // ------------2.Check if the incoming trajectory message pointer is empty--------------
    if (!msg)
    {
      error_string_tmp = "Received null-pointer trajectory message, skipping.";
      ROS_WARN_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }

    // ------------3.Obtain time data--------------
    // 3.1 Read time data
    typename Base::TimeData *time_data = this->time_data_.readFromRT();
    // 3.2 Calculate the time for the next update
    const ros::Time next_update_time = time_data->time + time_data->period;
    // 3.3 calculatedownoneNext updatenewofpositiveoftentransportlinetime
    ros::Time next_update_uptime = time_data->uptime + time_data->period;

    // ------------4.Check if the trajectory is empty--------------
    if (msg->points.empty())
    {
      this->setHoldPosition(time_data->uptime, gh);
      ROS_DEBUG_NAMED(this->name_, "Empty trajectory command, stopping.");
      return true;
    }

    // ------------5.Set trajectory initialization options--------------
    // 5.1 Trajectory initialization option declaration
    TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    // 5.2 Settings options
    options.other_time_base = &next_update_uptime;
    options.current_trajectory = curr_traj_ptr.get();
    options.joint_names = &this->joint_names_;
    options.angle_wraparound = &this->angle_wraparound_;
    options.rt_goal_handle = gh;
    options.default_tolerances = &this->default_tolerances_;
    options.allow_partial_joints_goal = this->allow_partial_joints_goal_;

    // ------------6.morenewwhenfrontpositiveinholdlineoftrajectory--------------
    //////////////////Real time update of trajectory/////////////////////
    try
    {
      TrajectoryPtr traj_ptr(new Trajectory);
      *traj_ptr = joint_trajectory_controller::
          initJointTrajectory<Trajectory>(*msg, next_update_time, options);
      if (!traj_ptr->empty())
      { // Set the new trajectory as the current trajectory
        this->curr_trajectory_box_.set(traj_ptr);
      }
      else
      {
        return false;
      }
    }
    /////////////////////////////////////////////////
    // catch exception
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM_NAMED(this->name_, ex.what());
      options.setErrorString(ex.what());
      return false;
    }
    catch (...)
    {
      error_string_tmp = "Unexpected exception caught when initializing "
                         "trajectory from ROS message data.";
      ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
      options.setErrorString(error_string_tmp);
      return false;
    }
    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      trajectoryCommandCB(const JointTrajectoryConstPtr &msg)
  {
    // invoke updateTrajectoryCommand Function to update trajectory command
    const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
    // If the trajectory update is successful，Terminate whenfrontofactivityeyemark
    if (update_ok)
    {
      preemptActiveGoal();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      dualtrajectoryCommandCB(const JointTrajectoryConstPtr &msg)
  {
    trajectory_msgs::JointTrajectoryConstPtr single_msg;
    // Confirm whenfrontThe controller is left/Right robotic arm------------------
    if (this->robot_name_ == "left_robot")
      single_msg = processTrajectory(msg, false);
    else if (this->robot_name_ == "right_robot")
      single_msg = processTrajectory(msg, true);
    // Confirm whenfrontThe controller is left/Right robotic arm------------------

    // invoke updateTrajectoryCommand Function to update trajectory command
    const bool update_ok = updateTrajectoryCommand(single_msg, RealtimeGoalHandlePtr());
    // If the trajectory update is successful，Terminate whenfrontofactivityeyemark
    if (update_ok)
    {
      preemptActiveGoal();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::preemptActiveGoal()
  {
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    // checkwhenfrontactivityeyemark
    if (current_active_goal)
    {
      // Reset and Cancel Activity Objectives
      this->rt_active_goal_.reset();
      current_active_goal->gh_.setCanceled();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<
      SegmentImpl, HardwareInterface>::goalCB(GoalHandle gh)
  {
    ROS_DEBUG_STREAM_NAMED(this->name_, "Received new action goal");

    // ------------1.Processing controldevicenot yettransportlineofsituation--------------
    // checkIs the controller availableintransportline
    if (!this->isRunning())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Can't accept new action goals. Controller is not running.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; // TODO: Add better error status to msg?
      gh.setRejected(result);
      return;
    }

    // ------------2.Dealing with situations where some targets are not allowed--------------
    // Partial objectives（Partial joint targets）Is it allowed
    if (!this->allow_partial_joints_goal_)
    {
      if (gh.getGoal()->trajectory.joint_names.size() !=
          this->joint_names_.size())
      {
        ROS_ERROR_NAMED(this->name_,
                        "Joints on incoming goal don't match the controller joints.");
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(result);
        return;
      }
    }

    // ------------3.Dealing with joint name mismatches--------------
    using joint_trajectory_controller::internal::mapping;
    std::vector<unsigned int> mapping_vector =
        mapping(gh.getGoal()->trajectory.joint_names, this->joint_names_);
    // Target joint nameandController mismatch
    if (mapping_vector.empty())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Joints on incoming goal don't match the controller joints.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }

    // ------------4.Attempt to update new trajectory targets and process related feedback--------------
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
    std::string error_string;
    // Attempt to update trajectory（update_okIs the update successful）
    const bool update_ok = updateTrajectoryCommand(
        joint_trajectory_controller::internal::share_member(
            gh.getGoal(), gh.getGoal()->trajectory),
        rt_goal, &error_string);
    // SettingsfeedbackChineseofJoint name
    rt_goal->preallocated_feedback_->joint_names = this->joint_names_;

    // ------------5.according toupdate_okDeciding whether to accept or reject a new goal--------------
    if (update_ok)
    {
      // Accept new goals
      preemptActiveGoal();
      gh.setAccepted();
      this->rt_active_goal_ = rt_goal;

      // Set target status check timer
      this->goal_handle_timer_ = this->controller_nh_.createTimer(
          this->action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
      this->goal_handle_timer_.start();
    }
    else
    {
      // Processing morenewfailureofsituation
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      result.error_string = error_string;
      gh.setRejected(result);
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  void SelfDefinedTrajectoryController<
      SegmentImpl, HardwareInterface>::cancelCB(GoalHandle gh)
  {
    // 1.Create WhenfrontactivityeyemarkofReal time handle
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // 2.Check if the cancellation request is targeted towards the current activity objective
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
      // 2.1 Reset whenfronteyemark
      this->rt_active_goal_.reset();
      // 2.2 Obtain the running time of the controller
      const ros::Time uptime = this->time_data_.readFromRT()->uptime;
      // 2.3 entermaintainwhenfrontpositionpattern
      this->setHoldPosition(uptime);
      ROS_DEBUG_NAMED(this->name_, "Canceling active action goal because cancel "
                                   "callback recieved from actionlib.");
      // 2.4 Translate into EnglishwhenfronteyeMarking marksforCanceled
      current_active_goal->gh_.setCanceled();
    }
  }

  template <class SegmentImpl, class HardwareInterface>
  bool SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      queryStateService(control_msgs::QueryTrajectoryState::Request &req,
                        control_msgs::QueryTrajectoryState::Response &resp)
  {
    // 1.Check if the controller is running
    if (!this->isRunning())
    {
      ROS_ERROR_NAMED(this->name_,
                      "Can't sample trajectory. Controller is not running.");
      return false;
    }

    // 2.Translate into EnglishrequesttimetransformationInternal monotonic representation
    typename Base::TimeData *time_data = this->time_data_.readFromRT();
    const ros::Duration time_offset = req.time - time_data->time;
    const ros::Time sample_time = time_data->uptime + time_offset;

    // 3.obtainwhenfronttrajectory
    TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    Trajectory &curr_traj = *curr_traj_ptr;

    // 4.initializationresponsepointstatus
    typename Segment::State response_point =
        typename Segment::State(this->joint_names_.size());

    // 5.Sampling trajectory status
    for (unsigned int i = 0; i < this->getNumberOfJoints(); ++i)
    {
      typename Segment::State state;
      typename TrajectoryPerJoint::const_iterator segment_it =
          sample(curr_traj[i], sample_time.toSec(), state);
      if (curr_traj[i].end() == segment_it)
      {
        ROS_ERROR_STREAM_NAMED(
            this->name_, "Requested sample time precedes trajectory start time.");
        return false;
      }

      response_point.position[i] = state.position[0];
      response_point.velocity[i] = state.velocity[0];
      response_point.acceleration[i] = state.acceleration[0];
    }

    // 6.Fill in response message
    resp.name = this->joint_names_;
    resp.position = response_point.position;
    resp.velocity = response_point.velocity;
    resp.acceleration = response_point.acceleration;

    return true;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline void
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      admittance_control(const ros::Duration &period)
  {
    // -----------------------------Admittance controller------------------------------------------
    // ------------6.Reading torque error---------------
    error_wrench_ = *(error_wrench_rb_.readFromRT());
    // std::cout << "error_wrench : " << error_wrench_ << std::endl;

    // ////////////////////////////////////////////////////////////
    // // Adaptive impedance
    // Adaptive_Damp_Z(period);
    // Vector6D tmp_dD;
    // for (size_t i = 0; i < tmp_dD.size(); i++)
    //   tmp_dD[i] = dD_value[i];
    // Eigen::Matrix<double, 6, 6> d_D;
    // d_D = tmp_dD.asDiagonal();
    // ////////////////////////////////////////////////////////////

    // ------------7.Admittance controlleralgorithm--------------
    // (Because the joint state used is from the previous cycle，thereforex_eanddx_eTo add desired_***_*period.toSec())
    // 7.1 whenfrontendpositionandexpectationendpositionofgap(Only caused by external forces,Not considering trajectory tracking)
    // pinocchio::log6(curr_pose_.actInv(desired_pose_)）: desired_pose_incurr_pose_The representation below
    // curr_pose_.act(-pinocchio::log6(curr_pose_.actInv(desired_pose_))): curr_pose_ - desired_pose_'
    x_e_ = curr_pose_.act(-pinocchio::log6(curr_pose_.actInv(desired_pose_))) +
           desired_vel_ * period.toSec(); // pinocchio::Motion (Finally, refer tocurr_pose_placeinofbase_linkcoordinate system)

    // std::cout << "---x_e_---" << x_e_.toVector().transpose() << std::endl;
    // 7.2 whenfrontendspeedandexpectationendspeedofgap（Only caused by external forces,Not considering trajectory tracking）
    dx_e_ = pinocchio::Motion(curr_vel_ - desired_vel_ +
                              desired_acc_ * period.toSec());
    // 7.3 Admittance control rate（Only caused by external forces,Not considering trajectory tracking）,calculateEnd addspeed
    ddx_e_ = pinocchio::Motion(M_.inverse() *
                               (error_wrench_ - K_ * x_e_.toVector() - D_ * dx_e_.toVector()));

    // ------------8.speed/Acceleration scaling--------------
    // 8.1 Acceleration scaling,Avoid being too large
    double traAccNorm = ddx_e_.linear().norm(); // Translational acceleration
    if (traAccNorm > traAccLimit_)
    {
      ddx_e_ = ddx_e_ * traAccLimit_ / traAccNorm;
      ROS_WARN_STREAM("traAccNorm: " << traAccNorm << " is larger than " << traAccLimit_);
    }
    double rotAccNorm = ddx_e_.angular().norm(); // rotateplusspeed
    if (rotAccNorm > rotAccLimit_)
    {
      ddx_e_ = ddx_e_ * rotAccLimit_ / rotAccNorm;
      ROS_WARN_STREAM("rotAccNorm: " << rotAccNorm << " is larger than " << rotAccLimit_);
    }
    // 8.2 calculateendexpectationspeed------------------（Final outputoneaendexpectationspeedexpectation）
    desired_vel_ = curr_vel_ + (desired_acc_ + ddx_e_) * period.toSec();
    // 8.3 speedzoom,avoidspeedtoo large
    double traVelNorm = desired_vel_.linear().norm(); // *translationspeed
    if (traVelNorm > traVelLimit_)
    {
      desired_vel_ = desired_vel_ * traVelLimit_ / traVelNorm;
      ROS_WARN_STREAM("traVelNorm: " << traVelNorm << " is larger than " << traVelLimit_);
    }
    double rotVelNorm = desired_vel_.angular().norm(); // *rotatespeed
    if (rotVelNorm > rotVelLimit_)
    {
      desired_vel_ = desired_vel_ * rotVelLimit_ / rotVelNorm;
      ROS_WARN_STREAM("rotVelNorm: " << rotVelNorm << " is larger than " << rotVelLimit_);
    }

    // ------------9.calculaterobotexpectationstatus--------------(Joint spaceqpoints)
    // 8.1 calculatewhenfrontstatusdownofJacobimatrix
    J_.setZero();
    pinocchio::computeFrameJacobian(robot_model_, model_curr_data_,
                                    pino_curr_pos_, compliance_frameId_,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J_);
    // 8.2 calculateexpectationjointspeed（Terminal Cartesian coordinates——>Joint space）
    pino_desired_vel_ = J_.completeOrthogonalDecomposition().pseudoInverse() *
                        desired_vel_.toVector();
    // 8.3 calculateexpectationJoint angle（based onpino_curr_pos_andupDescribe and obtainpino_desired_vel_）
    pino_desired_pos_ = pinocchio::integrate(robot_model_, pino_curr_pos_,
                                             pino_desired_vel_ * period.toSec());
    // 8.4 Translate into EnglishexpectationstatusAssign todesired_state_As an output
    for (size_t i = 0; i < this->joints_.size(); ++i)
    {
      this->desired_state_.position[i] = pino_desired_pos_(i);
      this->desired_state_.velocity[i] = pino_desired_vel_(i);
      // pino_desired_acc_unassigned ，maintainzero
      this->desired_state_.acceleration[i] = pino_desired_acc_(i);
    }
    // ---------------------------------end-------------------------------------------
    // ------------getdesired_state_，Then it will be sent to the control hardware
  }

  template <class SegmentImpl, class HardwareInterface>
  inline double*
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      Adaptive_Damp_Z(const ros::Duration &period)
  {
    // 1.obtaintoolUnder the coordinate systemoferror_wrench
    pinocchio::Force error_current;
    error_current = error_wrench_;
    error_current = model_temp_data_.oMf[compliance_frameId_].actInv(error_current);

    // assignment
    double Damp_proportion[6], Damp_integral[6], Damp_difference[6];

    for(int i=0; i<6; i++)
    { 
      if(i < 3){
        error_zforce[i][0] = error_zforce[i][1];
        error_zforce[i][1] = error_zforce[i][2];
        error_zforce[i][2] = error_current.linear()(i);
      }
      else
      {
        error_zforce[i][0] = error_zforce[i][1];
        error_zforce[i][1] = error_zforce[i][2];
        error_zforce[i][2] = error_current.angular()(i-3);
      }

      // incrementalpid
      Damp_proportion[i] = ((error_zforce[i][2] - error_zforce[i][1]) * AD_kp);
      Damp_integral[i] = (error_zforce[i][2] * AD_ki * 0.01);
      Damp_difference[i] = ((error_zforce[i][2] + error_zforce[i][0] - 2 * error_zforce[i][1]) * AD_kd);

      // pointsamplitude limiting（Anti integral saturation）
      if (fabs(Damp_integral[i]) >= 9 * 80 / 10)
        if (Damp_integral[i] > 0)
          Damp_integral[i] = 9 * 80 / 10;
        else
          Damp_integral[i] = -9 * 80 / 10;

      dD_value[i] += (Damp_proportion[i] + Damp_integral[i] + Damp_difference[i]);

      // Motor limit
      if (abs(dD_value[i]) >= 0.95 * 80)
      {
        if (dD_value[i] > 0)
          dD_value[i] = 0.95 * 80;
        else
          dD_value[i] = -0.95 * 80;
      }

    }
    // if(this->robot_name_ == "left_robot")
    //   std::cout << "dD_value: " << dD_value[0] <<" " <<  dD_value[1] << " "<< dD_value[2] << " "
    //                             << dD_value[3] <<" " <<  dD_value[4] << " "<< dD_value[5]
    //                             << std::endl;
    return dD_value;
  }

  template <class SegmentImpl, class HardwareInterface>
  inline std::string
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      getParentNamespace(const ros::NodeHandle &nh, int level)
  {
    std::string ns = nh.getNamespace(); // obtainComplete namespace
    if (ns.empty() || ns[0] != '/')
    {
      ns = "/" + ns; // Ensure to '/' beginning
    }

    // press '/' Split namespace
    std::vector<std::string> tokens;
    std::stringstream ss(ns);
    std::string token;
    while (std::getline(ss, token, '/'))
    {
      if (!token.empty())
      {
        tokens.push_back(token);
      }
    }

    // checkIs the hierarchy effective
    if (level <= 0 || level > static_cast<int>(tokens.size()))
    {
      return ""; // If the hierarchy is invalid，Return an empty string
    }

    return tokens[level - 1]; // Return the namespace of the specified level
  }

  template <class SegmentImpl, class HardwareInterface>
  inline trajectory_msgs::JointTrajectoryPtr
  SelfDefinedTrajectoryController<SegmentImpl, HardwareInterface>::
      processTrajectory(const trajectory_msgs::JointTrajectoryConstPtr &msg, bool use_last_six)
  {
    // createoneanewof JointTrajectory message
    trajectory_msgs::JointTrajectoryPtr single_msg(new trajectory_msgs::JointTrajectory);
    // Determine the starting and ending indexes that need to be extracted
    const size_t total_joints = msg->joint_names.size();
    const size_t start_index = use_last_six ? total_joints - 6 : 0; // front6a or after6a
    const size_t end_index = start_index + 6;
    // Copy header information
    single_msg->header = msg->header;
    // Copy the required joint names
    single_msg->joint_names.insert(single_msg->joint_names.end(),
                                   msg->joint_names.begin() + start_index,
                                   msg->joint_names.begin() + end_index);
    // traverse msg ofeachatrajectorypoint
    for (const auto &point : msg->points)
    {
      // createoneanewof TrajectoryPoint
      trajectory_msgs::JointTrajectoryPoint single_point;
      // Extract the required joint values
      single_point.positions.insert(single_point.positions.end(),
                                    point.positions.begin() + start_index,
                                    point.positions.begin() + end_index);
      if (!point.velocities.empty())
      {
        single_point.velocities.insert(single_point.velocities.end(),
                                       point.velocities.begin() + start_index,
                                       point.velocities.begin() + end_index);
      }
      if (!point.accelerations.empty())
      {
        single_point.accelerations.insert(single_point.accelerations.end(),
                                          point.accelerations.begin() + start_index,
                                          point.accelerations.begin() + end_index);
      }
      if (!point.effort.empty())
      {
        single_point.effort.insert(single_point.effort.end(),
                                   point.effort.begin() + start_index,
                                   point.effort.begin() + end_index);
      }
      // retention time
      single_point.time_from_start = point.time_from_start;
      // add to single_msg The trajectory point
      single_msg->points.push_back(single_point);
    }
    // Return processingafteroftrajectorymessage
    return single_msg;
  }

} // namespace selfdefined_controllers
