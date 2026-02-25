#include "manipulator/robot_arm.h"

namespace Robot_capsulation
{
    bool Robot_operation::loadSpoonParams(ros::NodeHandle& nh, const std::string& prefix, FuzzyParams& params) {
        // loadparameter
        bool success = true;
        success &= nh.getParam(prefix + "_a_min", params.a_min);
        success &= nh.getParam(prefix + "_e_param_1", params.e[0]);
        success &= nh.getParam(prefix + "_e_param_2", params.e[1]);
        success &= nh.getParam(prefix + "_e_param_3", params.e[2]);
        success &= nh.getParam(prefix + "_ec_param_1", params.ec[0]);
        success &= nh.getParam(prefix + "_ec_param_2", params.ec[1]);
        success &= nh.getParam(prefix + "_ec_param_3", params.ec[2]);
        success &= nh.getParam(prefix + "_a_param_1", params.a[0]);
        success &= nh.getParam(prefix + "_a_param_2", params.a[1]);
        success &= nh.getParam(prefix + "_a_param_3", params.a[2]);
        success &= nh.getParam(prefix + "_full",params.full);
        
        if (success)
            ROS_INFO_STREAM(prefix << " params loaded: "
                << "e=[" << params.e[0] << "," << params.e[1] << "," << params.e[2] << "] "
                << "ec=[" << params.ec[0] << "," << params.ec[1] << "," << params.ec[2] << "] "
                << "a=[" << params.a[0] << "," << params.a[1] << "," << params.a[2] << "]"
                << "full=" << params.full << "," << params.a_min);
        else ROS_ERROR_STREAM("Failed to load " << prefix << " params");
        return success;
    }
    /*------------------------Single arm area--------------------------------*/
    Robot_operation::Robot_operation(ros::NodeHandle &nh,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_dual,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_left,
                                     moveit::planning_interface::MoveGroupInterfacePtr mgtr_right,
                                     moveit_visual_tools::MoveItVisualToolsPtr vtptr): 
                                     robot_tfBuffer_(), robot_tfListener_(robot_tfBuffer_)
    {
        // Robot Status Subscription
        robot_service_ = nh_.advertiseService("robot", &Robot_operation::query_Callback, this);
        // Set dual arm status
        MSGptr_.reset(new MSG());
        MSGptr_->state = "error";
        //////////////////////////////////////////////////////////////////////////////////////////
        // readDashboard_flagparameter
        if (nh.getParam("A_manipulator/Dashboard_flag", Dashboard_flag))
            ROS_INFO("Got param: %d", Dashboard_flag);
        else
            ROS_ERROR("Failed to get param 'Dashboard_flag'");
        // readBalance_flagparameter
        if (nh.getParam("A_manipulator/Balance_flag", Balance_flag))
            ROS_INFO("Got param: %d", Balance_flag);

        // readR_RGIC_flagparameter
        if (nh.getParam("A_manipulator/R_RGIC_flag", R_RGIC_flag))
            ROS_INFO("Got param: %d", R_RGIC_flag);
        else
            ROS_ERROR("Failed to get param 'PGI140_flag'");
        // readL_PGI140_flagparameter
        if (nh.getParam("A_manipulator/L_PGI140_flag", L_PGI140_flag))
            ROS_INFO("Got param: %d", L_PGI140_flag);
        else
            ROS_ERROR("Failed to get param 'PGI140_flag'");
        if (nh.getParam("A_manipulator/ADP1000_flag", ADP1000_flag))
            ROS_INFO("Got param: %d", ADP1000_flag);
        else
            ROS_ERROR("Failed to get param 'ADP1000_flag'");    
        if (nh.getParam("A_manipulator/Locator_flag", Locator_flag))
            ROS_INFO("Got param: %d", Locator_flag);
        else
            ROS_ERROR("Failed to get param 'Locator_flag'");    
        // readdebug_enable_flagparameter
        if (nh.getParam("A_manipulator/debug_enable_flag", debug_enable_flag))
            ROS_INFO("Got param: %d", debug_enable_flag);
        else
            ROS_ERROR("Failed to get param 'debug_enable_flag'");
        // readAutostrech_flagparameter
        if (nh.getParam("A_manipulator/Autostrech_flag", Autostrech_flag))
            ROS_INFO("Got param: %d", Autostrech_flag);
        else
            ROS_ERROR("Failed to get param 'Autostrech_flag'");
        
        // readLoad_JPfileparameter
        std::string Load_JPfile_name = "gelrobot";
        if (nh.getParam("A_manipulator/Load_JPfile", Load_JPfile_name))
            ROS_INFO("Load JPfile name: %s", Load_JPfile_name.c_str());
        else
            ROS_ERROR("Failed to load JPfile 'Load_JPfile'");

        // readAutostrech_flagparameter
        if (nh.getParam("A_manipulator/Image_flag", Image_flag))
            ROS_INFO("Got param: %d", Image_flag);
        else
            ROS_ERROR("Failed to get param 'Image_flag'");

        if (Dashboard_flag)
        {
            left_dbptr = std::make_shared<dashboardsrv_client>(nh, "/left_robot");
            right_dbptr = std::make_shared<dashboardsrv_client>(nh, "/right_robot");

            left_dbptr->robot_init();
            left_dbptr->load_program("pro_1.urp\n");
            left_dbptr->stop();
            left_dbptr->play();
            left_dbptr->DO_init();

            right_dbptr->robot_init();
            right_dbptr->load_program("pro_2.urp\n");
            right_dbptr->stop();
            right_dbptr->play();
            right_dbptr->DO_init();
        }
        //clamp jawmodbus
        //balancemodbus
        if(Balance_flag){
            ros::service::waitForService("/Balance_com");
            balance_client = nh.serviceClient<balance_com::Balance>("/Balance_com");
        }

        //Fuzzy controller parameters
        bool smallSuccess = loadSpoonParams(nh, "A_manipulator/S", smallSpoonParams);
        bool bigSuccess = loadSpoonParams(nh, "A_manipulator/B", bigSpoonParams);
        if (!smallSuccess || !bigSuccess)ROS_ERROR("Failed to load all spoon parameters");

        if (R_RGIC_flag)
        {
            ros::service::waitForService("/right_gripper_control");
            right_gripper_client = nh.serviceClient<gripper_modbus::Gripper>("/right_gripper_control");
            std::string R_RGIC_port = "ttyUSB0";
            if (nh.getParam("A_manipulator/R_RGIC_port", R_RGIC_port))
                ROS_INFO("R_RGIC port: %s", R_RGIC_port.c_str());
            else
                ROS_WARN("Failed to get param 'A_manipulator/R_RGIC_port'.");
        }
        if (L_PGI140_flag)
        {
            ros::service::waitForService("/left_gripper_control");
            left_gripper_client = nh.serviceClient<gripper_modbus::Gripper>("/left_gripper_control");
            std::string L_PGI140_port = "ttyUSB1";
            if (nh.getParam("A_manipulator/L_PGI140_port", L_PGI140_port))
                ROS_INFO("L_PGI140_port port: %s", L_PGI140_port.c_str());
            else
                ROS_WARN("Failed to get param 'A_manipulator/L_PGI140_port'.");
        }
        if(ADP1000_flag||true)
        {
            adp1000_pub = nh.advertise<std_msgs::Int32>("/adp1000cmd", 10);
        }
        if (Locator_flag)
        {
            ros::service::waitForService("/locator_service");
            locator_topic_pub = nh.advertise<std_msgs::String>("/locator_topic", 10);
            locator_client = nh.serviceClient<locatornew::LocatorService>("/locator_service");
        }
        bottle_weight=0;
        tare_force_z_pub = nh.advertise<std_msgs::Float64>("/tare_force_z", 10);
        std_msgs::Float64 msg;
        msg.data=bottle_weight=-50;
        tare_force_z_pub.publish(msg);
        left_twist_pub = nh.advertise<std_msgs::Float64MultiArray>("/left_robot/twist_param", 10);
        right_twist_pub = nh.advertise<std_msgs::Float64MultiArray>("/right_robot/twist_param", 10);
        //////////////////////////////////////////////////////////////////////////////////////////

        this->nh_ = nh;
        this->visual_tools_ptr_ = vtptr;

        move_group_ptr = mgtr_dual;
        dual_robot_ptr = std::make_shared<Robot_arm::Dual_arm>(nh, move_group_ptr, visual_tools_ptr_);
        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_ptr->setPlannerId("PTP");

        left_mgtr = mgtr_left;
        left_robot_ptr = std::make_shared<Robot_arm::Single_arm>(nh, left_mgtr, visual_tools_ptr_);
        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
        left_mgtr->setPlannerId("PTP");
        // left_mgtr->setPlanningPipelineId("ompl");
        // left_mgtr->setPlannerId("RRTConnect");


        right_mgtr = mgtr_right;
        right_robot_ptr = std::make_shared<Robot_arm::Single_arm>(nh, right_mgtr, visual_tools_ptr_);
        right_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
        right_mgtr->setPlannerId("PTP");
        // right_mgtr->setPlanningPipelineId("ompl");
        // right_mgtr->setPlannerId("RRTConnect");

        // Activate power conversion
        ft_transition_ptr = std::make_shared<Operate_tool::FT_transition>(nh);

        // Release the target torque for the right arm
        right_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(
            "/right_robot/target_wrench", 1);
        // Publish the target torque of the left arm
        left_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(
            "/left_robot/target_wrench", 1);

        // Post the topic of left arm force control
        left_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/left_robot/selfdefined_trajectory_controller/command", 1);
        // Post the topic of right arm force control
        right_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/right_robot/selfdefined_trajectory_controller/command", 1);

        // Post the topic of dual arm force control
        dual_controller_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/dual_robot/selfdefined_trajectory_controller/command", 1);

        // Workstation callback function
        station_sub_ = nh.subscribe<std_msgs::String>("/stationOperation_in", 1,
                                                       &Robot_operation::station_cb, this);
        station_pub_ = nh.advertise<std_msgs::String>("/stationOperation_out", 1);

        opera_over_pub_ = nh.advertise<std_msgs::String>("/stationOperation_over", 1);

        Record_tool::load_joint_pose("left_"+Load_JPfile_name+"_JP", joints_, abs_pose_, station_id_);
        Record_tool::load_joint_pose("right_"+Load_JPfile_name+"_JP", joints_, abs_pose_, station_id_);
        MSGptr_->state = "idle";
        if (Image_flag){
            drugdata_client = nh.serviceClient<image_deal::Drugdata>("Drugdata");
            // Waiting for the service to start successfully
            drugdata_client.waitForExistence();
            ROS_INFO("drugdata_client has launched.");
        }
    }

    Robot_operation::~Robot_operation()
    {
        ROS_INFO_NAMED("Robot_operation", "The operation of Robot is to free.....");
        if (Dashboard_flag)
        {
            left_dbptr->stop();
            right_dbptr->stop();
        }
    }

    void Robot_operation::station_cb(const std_msgs::StringConstPtr &obsOperation_in)
    {
        Json::Reader jsonreader;
        Json::Value root;
        jsonreader.parse(obsOperation_in->data, root);
        ROS_INFO_STREAM("--------------received command---------------\n"
                        << root.toStyledString());

        std::string station_name = root["curr_station"].asString();
        std::string operation = root["operation"].asString();
        std::string command = root["order"].asString();
        MSGptr_->id = station_name;
        MSGptr_->exper_no = operation;
        MSGptr_->state = "running";
        MSGptr_->detail = command;
        std::string result = Analysis_Command(command);
        if(operation_over_Flag)
        {
            std_msgs::String return_msg;
            if(~operation_over_Flag)
                return_msg.data = "{'cmd':'" + operation + "','isFinish':true}";
            else
                return_msg.data = "{'cmd':'" + operation + "','isFinish':false}";
            opera_over_pub_.publish(return_msg);
            operation_over_Flag = 0;
        }

        std_msgs::String return_msg;
        std::stringstream ss;
        ss << "{"
            << "\"curr_station\":\"" << station_name << "\","
            << "\"operation\":\"" << operation << "\","
            << "\"order\":\"" << command << "\","
            << "\"result\":\"" << result << "\""
            << "}";
        return_msg.data = ss.str();
        station_pub_.publish(return_msg);
        MSGptr_->state = "done";
        MSGptr_->detail = command + "has finished!";
        MSGptr_->state = "idle";
    }
    std::string Robot_operation::code_station_func_with_args(const std::string &operation, const std::vector<std::any>& args) {
        if (operation == "progress_over") {
            this->progress_over_flag = true;
        } else {
            if (station_func_with_args_map.find(operation) != station_func_with_args_map.end()) {
                return station_func_with_args_map[operation](args);
            } else {
                std::cout << "Function " << operation << " not found!" << std::endl;
            }
        }return "error";
    }
    std::string Robot_operation::code_station_func(const std::string &operation)
    {
        if (operation == "progress_over")
        {
            this->progress_over_flag = true;
        }
        else
        {
            if (station_func_map.find(operation) != station_func_map.end()) 
                return (this->*station_func_map[operation])();
            else 
                std::cout << "Function " << operation << " not found!" << std::endl;
        }return "error";
    }
    std::string Robot_operation::gripper_ctrl(
        ros::ServiceClient gripper_client, 
        int force, int velocity, int position, 
        int torque, int speed, int abs_angle, int rel_angle){
        // Create service request object
        gripper_modbus::Gripper srv;
        // Set request parameters
        srv.request.force               = force    ;//Parallel force value，20～100
        srv.request.velocity            = velocity ;//Parallel velocity，1～100
        srv.request.torque              = torque      ;//Rotation force value，20～100
        srv.request.speed               = speed   ;//rotation speed，1～100
        srv.request.abs_angle           = abs_angle         ;//Absolute angle，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = rel_angle         ;//relative angle，-32768～32767
        srv.request.position            = position          ;//Parallel position，0～1000
        srv.request.block_flag          = true              ;//Blocking sign
        srv.request.stop_flag           = false             ;//Force stop
        srv.request.reset_flag          = false             ;//initialization
        srv.request.feedback            = 0b1001001 ;       //Feedback options

        if (gripper_client.call(srv)) {
            ROS_INFO("Gripper status: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("Service call failed！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_ctrl_asyn(
        ros::ServiceClient gripper_client, 
        int force, int velocity, int position, 
        int torque, int speed, int abs_angle, int rel_angle){
        // Create service request object
        gripper_modbus::Gripper srv;
        // Set request parameters
        srv.request.force               = force    ;//Parallel force value，20～100
        srv.request.velocity            = velocity ;//Parallel velocity，1～100
        srv.request.torque              = torque      ;//Rotation force value，20～100
        srv.request.speed               = speed   ;//rotation speed，1～100
        srv.request.abs_angle           = abs_angle         ;//Absolute angle，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = rel_angle         ;//relative angle，-32768～32767
        srv.request.position            = position          ;//Parallel position，0～1000
        srv.request.block_flag          = false              ;//Blocking sign
        srv.request.stop_flag           = false             ;//Force stop
        srv.request.reset_flag          = false             ;//initialization
        srv.request.feedback            = 0b1001001 ;       //Feedback options

        if (gripper_client.call(srv)) {
            ROS_INFO("Gripper status: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("Service call failed！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_stop(ros::ServiceClient gripper_client){
        // Create service request object
        gripper_modbus::Gripper srv;
        // Set request parameters
        srv.request.force               = -1    ;//Parallel force value，20～100
        srv.request.velocity            = -1 ;//Parallel velocity，1～100
        srv.request.torque              = -1      ;//Rotation force value，20～100
        srv.request.speed               = -1   ;//rotation speed，1～100
        srv.request.abs_angle           = 99999999         ;//Absolute angle，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = 0         ;//relative angle，-32768～32767
        srv.request.position            = -1          ;//Parallel position，0～1000
        srv.request.block_flag          = true              ;//Blocking sign
        srv.request.stop_flag           = true             ;//Force stop
        srv.request.reset_flag          = false             ;//initialization
        srv.request.feedback            = 0b1001001 ;       //Feedback options

        if (gripper_client.call(srv)) {
            ROS_INFO("Gripper status: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("Service call failed！");
            return "offline";
        }
    }
    std::string Robot_operation::gripper_reset(ros::ServiceClient gripper_client){
        // Create service request object
        gripper_modbus::Gripper srv;
        // Set request parameters
        srv.request.force               = -1    ;//Parallel force value，20～100
        srv.request.velocity            = -1 ;//Parallel velocity，1～100
        srv.request.torque              = -1      ;//Rotation force value，20～100
        srv.request.speed               = -1   ;//rotation speed，1～100
        srv.request.abs_angle           = 99999999         ;//Absolute angle，-160*32768-32768～160*32678+32767
        srv.request.rel_angle           = 0         ;//relative angle，-32768～32767
        srv.request.position            = -1          ;//Parallel position，0～1000
        srv.request.block_flag          = true              ;//Blocking sign
        srv.request.stop_flag           = false             ;//Force stop
        srv.request.reset_flag          = true             ;//initialization
        srv.request.feedback            = 0b1001001 ;       //Feedback options

        if (gripper_client.call(srv)) {
            ROS_INFO("Gripper status: %s", srv.response.status.c_str());
            return srv.response.status;
        } else {
            ROS_ERROR("Service call failed！");
            return "offline";
        }
    }
    double Robot_operation::Balance_read(ros::ServiceClient balance_client){
        balance_com::Balance srv;
        srv.request.operation = "weight";
        if (balance_client.call(srv)) {
            ROS_INFO("balanceIndications:%.1lf",srv.response.weight);
            return srv.response.weight;
        } else {
            ROS_ERROR("Failed to call service for read weight operation");
            return -1.0;
        }
    }
    double Robot_operation::Balance_tare(ros::ServiceClient balance_client){
        balance_com::Balance srv;
        srv.request.operation = "tare";
        if (balance_client.call(srv)&&srv.response.success) {
            ROS_INFO("balancePeeling successful");
            return srv.response.weight;
        } else {
            ROS_ERROR("Failed to call service for tare operation");
            return -99999999;
        }
    }
    std::string Robot_operation::GetStamp()
    {
        long long stamp = ros::Time::now().toNSec() / 1000000;
        return std::to_string(stamp);
    }
    bool Robot_operation::query_Callback(
        aichem_msg_srv::DmsService::Request &req,
        aichem_msg_srv::DmsService::Response &res)
    {
        std::string msg = (std::string)req.data;

        Json::Value root;
        root["id"] = MSGptr_->id;
        root["exper_no"] = MSGptr_->exper_no;
        root["stamp"] = GetStamp();
        root["state"] = MSGptr_->state;
        root["detail"] = MSGptr_->detail;

        res.data = root.toStyledString();
        return true;
    }
}
