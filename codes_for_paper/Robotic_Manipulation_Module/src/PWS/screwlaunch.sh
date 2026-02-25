#!/bin/zsh

# start roscore
gnome-terminal --tab -- zsh -c  "roscore; exec zsh"
sleep 2

# startURRobot driven
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch ur_robot_driver dual_ur_bringup.launch body_arm_flag:=true; exec zsh"
sleep 5

# Activate dual robotsMoveItconfiguration
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch dual_moveit_config dual_moveit_planning_execution.launch; exec zsh"

# # startRViz
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch manipulator dual_arm_rviz.launch; exec zsh"
# sleep 5

# # Activate the pipette
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; rosrun adp1000 adp1000.py; exec zsh"
# sleep 5

# # Activate Claw Service
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch gripper_modbus Gripper_ModbusControl.launch ; exec zsh"
# Activate balance communication
# gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch balance_com Balance_com.launch ; exec zsh"

#sudo udevadm trigger
#ll /dev/ |grep ttyUSB

# Release end effectortf
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; rosrun manipulator tool_end_tf_publisher.py ; exec zsh"
sleep 5

# startA_gel_start
gnome-terminal --tab -- zsh -c "source ~/B_workspace/screw_dual_robot/devel/setup.zsh; roslaunch manipulator A_robot_start.launch Dashboard_flag:=true ; exec zsh"
sleep 20

source ~/B_workspace/screw_dual_robot/devel/setup.zsh
roslaunch js_control js_control.launch switch_mode:=7 control_mode:=0
