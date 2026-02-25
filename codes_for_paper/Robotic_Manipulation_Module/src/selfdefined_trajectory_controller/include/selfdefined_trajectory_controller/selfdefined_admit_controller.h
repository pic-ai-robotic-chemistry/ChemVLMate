#pragma once

// pinocchiolibrary file
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/QR>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "selfdefined_trajectory_controller/selfdefinedControllerConfig.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#define EPSILON 1e-5

namespace selfdefined_controllers
{
  // Define Template ClassSelfDefinedTrajectoryController，Inherited from
  // joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
  template <class SegmentImpl, class HardwareInterface>
  class SelfDefinedTrajectoryController
      : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
  {
  public:
    // Both construction and analysis are default
    SelfDefinedTrajectoryController() = default;
    virtual ~SelfDefinedTrajectoryController() = default;

    // Controller initialization
    bool init(HardwareInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    // Controller update: current momenttime, Controller updateperiod2ms
    void update(const ros::Time &time, const ros::Duration &period);

  protected:
    typedef Eigen::Matrix<double, 6, 1> Vector6D;
    // ------------------------Simplify citations------------------------------------
    // Simplify the base classofquote
    using Base = joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
    // Base::ActionServerequivalent to
    // joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::ActionServer
    using typename Base::ActionServer;
    using typename Base::ActionServerPtr;
    using typename Base::GoalHandle;
    using typename Base::HwIfaceAdapter;
    using typename Base::JointHandle;
    using typename Base::JointTrajectoryConstPtr;
    using typename Base::RealtimeGoalHandle;
    using typename Base::RealtimeGoalHandlePtr;
    using typename Base::Scalar;
    using typename Base::Segment;
    using typename Base::StatePublisher;
    using typename Base::StatePublisherPtr;
    using typename Base::Trajectory;
    using typename Base::TrajectoryBox;
    using typename Base::TrajectoryPerJoint;
    using typename Base::TrajectoryPerJointPtr;
    using typename Base::TrajectoryPtr;
    // ------------------------Simplify citations------------------------------------

    // ---------------------Control algorithm function--------------------------------
    void admittance_control(const ros::Duration &period);
    double error_zforce[6][3], dD_value[6];
    double AD_kp, AD_ki, AD_kd;
    double* Adaptive_Damp_Z(const ros::Duration &period);
    double Coor_para;
    // ---------------------Control algorithm function--------------------------------

  private:
    std::string robot_name_;
    // ----------pinocchioDynamics libraryofData members----------
    // "performance"Smooth and compliant propertiesofjointid
    pinocchio::FrameIndex compliance_frameId_;
    // Robot arm modelofcurrent\Expectations and Temporary Data
    pinocchio::Data model_curr_data_, model_desired_data_, model_temp_data_;
    // robot model
    pinocchio::Model robot_model_;
    // Current pose and expected pose
    pinocchio::SE3 curr_pose_, desired_pose_;
    // Current speed,expected speed,Expected acceleration
    pinocchio::Motion curr_vel_, desired_vel_, desired_acc_;
    // Pose error,speed error,plusspeed error
    pinocchio::Motion x_e_, dx_e_, ddx_e_;
    // ----------pinocchioDynamics libraryofData members----------

    // --------------Motion restriction parameters----------------
    // Whether to print debugging content
    bool verbose_;
    // wrench dead zone
    std::vector<double> wrenchDead_;
    // Maximum translation speed limit traslation velocity limit
    double traVelLimit_;
    // Maximum limit of rotation speed rotation velocity limit
    double rotVelLimit_;
    // Maximum limit of translational acceleration
    double traAccLimit_;
    // Maximum limit of rotational acceleration
    double rotAccLimit_;
    // --------------Motion restriction parameters----------------

    // ----------------------Admittance controller parameters------------------------
    // Jacobimatrix
    Eigen::MatrixXd J_;
    // "performance"Smooth and compliant propertiesofjointofname
    std::string compliance_link_;
    // In admittance controlM,D,Kmatrix
    Eigen::Matrix<double, 6, 6> M_, D_, K_;
    // Current joint angle and joint angular velocity
    Eigen::VectorXd pino_curr_pos_, pino_curr_vel_;
    // Expected joint angle,jointangular velocityandjointcornerplusspeed
    Eigen::VectorXd pino_desired_pos_, pino_desired_vel_, pino_desired_acc_;

    // ---------------Dynamic configuration parameters----------------
    // Dynamic parameter configuration server,Used for dynamic adjustmentIn admittance controlM,D,K
    std::unique_ptr<dynamic_reconfigure::Server<
        selfdefined_trajectory_controller::selfdefinedControllerConfig>>
        dyn_conf_server_;
    void dynamicReconfigureCallback(
        selfdefined_trajectory_controller::selfdefinedControllerConfig &config, uint32_t level);
    // ---------------Dynamic configuration parameters----------------
    // ----------------------Admittance controller parameters------------------------

    // ------------Custom Function Parameters----------------
    // Used for subscription goalwrench and currentwrench ofsubscriber
    ros::Subscriber target_wrench_sub_, current_wrench_sub_;
    // goalwrench and currentwrench oferror
    Vector6D error_wrench_;
    // currentwrench
    pinocchio::Force current_wrench_;
    // goalwrench
    pinocchio::Force target_wrench_;
    // real-timebuffer
    realtime_tools::RealtimeBuffer<Vector6D> error_wrench_rb_;
    // ------------Custom Function Parameters----------------
  };
} // namespace selfdefined_controllers

#include "selfdefined_trajectory_controller/selfdefined_trajectory_controller_impl.h"