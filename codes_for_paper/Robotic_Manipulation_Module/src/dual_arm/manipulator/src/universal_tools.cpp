#include "manipulator/universal_tools.h"

namespace Pose_Tool
{
    // Euler2Quaternionclass definition-------------------------
    double Euler2Quaternion::getQua_x(void)
    {
        return value_getQua_x;
    }
    double Euler2Quaternion::getQua_y(void)
    {
        return value_getQua_y;
    }
    double Euler2Quaternion::getQua_z(void)
    {
        return value_getQua_z;
    }
    // Member function definition
    double Euler2Quaternion::getQua_w(void)
    {
        return value_getQua_w;
    }

    Euler2Quaternion::Euler2Quaternion(double X, double Y, double Z)
    {
        roll_deg = X * 180 / PI_euler;
        pitch_deg = Y * 180 / PI_euler;
        yaw_deg = Z * 180 / PI_euler;
    }

    void Euler2Quaternion::EulerXYZ(void)
    {
        value_getQua_x = cos(yaw_deg * PI_euler / 180.0 / 2) * cos(pitch_deg * PI_euler / 180.0 / 2) * sin(roll_deg * PI_euler / 180.0 / 2) + sin(yaw_deg * PI_euler / 180.0 / 2) * sin(pitch_deg * PI_euler / 180.0 / 2) * cos(roll_deg * PI_euler / 180.0 / 2);
        value_getQua_y = cos(yaw_deg * PI_euler / 180.0 / 2) * sin(pitch_deg * PI_euler / 180.0 / 2) * cos(roll_deg * PI_euler / 180.0 / 2) - sin(yaw_deg * PI_euler / 180.0 / 2) * cos(pitch_deg * PI_euler / 180.0 / 2) * sin(roll_deg * PI_euler / 180.0 / 2);
        value_getQua_z = sin(yaw_deg * PI_euler / 180.0 / 2) * cos(pitch_deg * PI_euler / 180.0 / 2) * cos(roll_deg * PI_euler / 180.0 / 2) + cos(yaw_deg * PI_euler / 180.0 / 2) * sin(pitch_deg * PI_euler / 180.0 / 2) * sin(roll_deg * PI_euler / 180.0 / 2);
        value_getQua_w = cos(yaw_deg * PI_euler / 180.0 / 2) * cos(pitch_deg * PI_euler / 180.0 / 2) * cos(roll_deg * PI_euler / 180.0 / 2) - sin(yaw_deg * PI_euler / 180.0 / 2) * sin(pitch_deg * PI_euler / 180.0 / 2) * sin(roll_deg * PI_euler / 180.0 / 2);
    }

    void Euler2Quaternion::change_XYZ(double new_X, double new_Y, double new_Z)
    {
        roll_deg = new_X * 180 / PI_euler;
        pitch_deg = new_Y * 180 / PI_euler;
        yaw_deg = new_Z * 180 / PI_euler;
    }
    // Euler2Quaternionclass definition-------------------------

    /*----------------------dataclasstypeturntransform----------------------------------*/
    // Convert quaternions toRPY
    std::vector<double> self_getRPY(geometry_msgs::Quaternion quaternion)
    {
        tf::Quaternion tf_quaternion;
        std::vector<double> RPY_angle;

        tf::quaternionMsgToTF(quaternion, tf_quaternion);
        tf::Matrix3x3 tf_matrix(tf_quaternion); // createonea3x3ofrotation matrix
        double roll, pitch, yaw;
        tf_matrix.getRPY(roll, pitch, yaw); // Obtain Euler angles
        // ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

        RPY_angle.push_back(roll);
        RPY_angle.push_back(pitch);
        RPY_angle.push_back(yaw);

        return RPY_angle;
    }

    Eigen::Isometry3d toMatrix(Eigen::Vector3d &X, Eigen::Quaterniond &Q)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.pretranslate(X);
        T.rotate(Q);
        return T;
    }

    Eigen::Isometry3d toMatrix(Eigen::Vector3d &X, Eigen::Vector3d &YPR)
    {
        Eigen::Quaterniond Q = Eigen::AngleAxisd(YPR[0], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(YPR[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(YPR[2], Eigen::Vector3d::UnitX());
        return toMatrix(X, Q);
    }

    Eigen::Isometry3d toMatrix(geometry_msgs::Pose &pose)
    {
        Eigen::Vector3d X;
        Eigen::Quaterniond Q;
        X[0] = pose.position.x;
        X[1] = pose.position.y;
        X[2] = pose.position.z;
        Q.coeffs()[0] = pose.orientation.x;
        Q.coeffs()[1] = pose.orientation.y;
        Q.coeffs()[2] = pose.orientation.z;
        Q.coeffs()[3] = pose.orientation.w;
        return toMatrix(X, Q);
    }

    geometry_msgs::Pose toPose(Eigen::Vector3d &X, Eigen::Quaterniond &Q)
    {
        geometry_msgs::Pose pose;
        pose.position.x = X[0];
        pose.position.y = X[1];
        pose.position.z = X[2];
        pose.orientation.x = Q.coeffs()[0];
        pose.orientation.y = Q.coeffs()[1];
        pose.orientation.z = Q.coeffs()[2];
        pose.orientation.w = Q.coeffs()[3];
        return pose;
    }

    geometry_msgs::Pose toPose(Eigen::Vector3d &X, Eigen::Vector3d &YPR)
    {
        Eigen::Quaterniond Q = Eigen::AngleAxisd(YPR[0], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(YPR[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(YPR[2], Eigen::Vector3d::UnitX());
        return toPose(X, Q);
    }

    geometry_msgs::Pose toPose(Eigen::Isometry3d &T)
    {
        Eigen::Vector3d X(T.translation());
        Eigen::Quaterniond Q(T.rotation());
        return toPose(X, Q);
    }
    /*----------------------dataclasstypeturntransform----------------------------------*/

    Eigen::Matrix3d rpy2R(const Eigen::Vector3d &rpy)
    {
        /**** 3. Euler angles ****/
        // 3.0 Initialize Euler angles(Z-Y-X，namelyRPY, First, go aroundxaxisroll,againwindyaxispitch,Finally, go aroundzaxisyaw
        // 3.1 Euler angle conversion to rotation matrix
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitX());
        return rotation_matrix3;
    }

    tf::Matrix3x3 matrix_ZYZ(double angle_z1, double angle_y1, double angle_z2)
    {
        tf::Matrix3x3 rotz1, roty1, rotz2;
        rotz1.setRPY(0, 0, angle_z1);
        roty1.setRPY(0, angle_y1, 0);
        rotz2.setRPY(0, 0, angle_z2);

        tf::Matrix3x3 result_matrix;
        result_matrix = rotz1 * roty1 * rotz2;

        return result_matrix;
    }

    geometry_msgs::Pose Pose_EulerRPY(const geometry_msgs::Pose& bowl_pose, 
                                      double EulerRPY_angle[3])
    {
        geometry_msgs::Pose goal_pose = bowl_pose;

        // Obtain the original quaternion
        tf2::Quaternion original_orientation;
        tf2::fromMsg(goal_pose.orientation, original_orientation);

        // Convert the original quaternion to Euler angles
        tf2::Matrix3x3 original_rotation(original_orientation);
        double original_roll, original_pitch, original_yaw;
        original_rotation.getRPY(original_roll, original_pitch, original_yaw);

        // Add Rotation：calculatenewofEuler angles
        double new_roll = original_roll + EulerRPY_angle[0];   // windXaxisofrotate
        double new_pitch = original_pitch + EulerRPY_angle[2]; // windYaxisofrotate
        double new_yaw = original_yaw + EulerRPY_angle[1];      // windZaxisofrotate

        // Translate into EnglishnewofEuler anglesConvert back to quaternions
        tf2::Quaternion new_orientation;
        new_orientation.setRPY(new_roll, new_pitch, new_yaw);

        // morenewpostureChineseofRotating part
        goal_pose.orientation = tf2::toMsg(new_orientation);

        return goal_pose;
    }

}

namespace Record_tool
{
    bool getJPDataModifiedParam(){
        bool value;
        if (ros::param::get("/B_record/JP_data_modified_flag", value))
            return value;
        else {
            ROS_WARN("Not foundtoparameter /B_record/JP_data_modified_flag，Use the default value: false");
            return false;
        }
    }
    void setJPDataModifiedParam(bool value){
        ros::param::set("/B_record/JP_data_modified_flag", value);
    }
    // Record joint angles
    bool writeFile(std::string file_path, std::string name,
                   std::vector<double> joints)
    {
        Json::Reader jsonreader;
        Json::Value root;
        Json::StyledWriter writer;
        std::ifstream ifile(file_path, std::ios::binary);
        if (jsonreader.parse(ifile, root))
        {
            int i;
            for (i = 0; i < root["joints"].size(); ++i)
            {
                if (root["joints"][i]["name"].asString() == name)
                    break;
            }
            Json::Value joint;
            joint["name"] = Json::Value(name);
            bool vaild_flag=false;
            for (int j = 0; j < 6; ++j)
            {
                joint["value"][j] = Json::Value(joints[j]);
                if(abs(joint["value"][j].asDouble())>1e-6)vaild_flag=true;
            }
            if(vaild_flag){
                if (i < root["joints"].size())
                {
                    root["joints"][i] = joint;
                    // std::cout << file_path << "\nModify joint angle:\n"
                    //           << joint.toStyledString() << std::endl;
                    std::cout << file_path << "\nModify joint angle:\n"
                            << joint["name"] << std::endl;
                }
                else
                {
                    root["joints"].append(joint);
                    // std::cout << file_path << "\nAdd joint angle:\n"
                    //           << joint.toStyledString() << std::endl;
                    std::cout << file_path << "\nAdd joint angle:\n"
                            << joint["name"] << std::endl;
                }
                setJPDataModifiedParam(true);
            }else{
                std::cout << file_path << "\njointcornerobtainabnormal:\n" << std::endl;
            }
            ifile.close();
            std::ofstream ofile(file_path, std::ios::binary);
            ofile << writer.write(root);
            ofile.close();
            return vaild_flag;
        }return false;
    }

    // Record pose
    bool writeFile(std::string file_path, std::string name,
                   geometry_msgs::Pose set_pose)
    {
        Json::Reader jsonreader;
        Json::Value root;
        Json::StyledWriter writer;
        std::ifstream ifile(file_path, std::ios::binary);
        if (jsonreader.parse(ifile, root))
        {
            int i;
            for (i = 0; i < root["poses"].size(); ++i)
            {
                if (root["poses"][i]["name"].asString() == name)
                    break;
            }
            Json::Value pose;
            pose["name"] = Json::Value(name);
            pose["value"][0] = Json::Value(set_pose.position.x);
            pose["value"][1] = Json::Value(set_pose.position.y);
            pose["value"][2] = Json::Value(set_pose.position.z);

            pose["value"][3] = Json::Value(set_pose.orientation.x);
            pose["value"][4] = Json::Value(set_pose.orientation.y);
            pose["value"][5] = Json::Value(set_pose.orientation.z);
            pose["value"][6] = Json::Value(set_pose.orientation.w);
            bool vaild_flag=false;
            for(int i=0;i<7;i++)
                if(abs(pose["value"][i].asDouble())>1e-6){
                    vaild_flag=true;
                    break;
                }
            if(vaild_flag){
                if (i < root["poses"].size())
                {
                    root["poses"][i] = pose;
                    // std::cout << file_path << "\nModify pose:\n"
                    //           << pose.toStyledString() << std::endl;
                    std::cout << file_path << "\nModify pose:\n"
                            << pose["name"] << std::endl;
                }
                else
                {
                    root["poses"].append(pose);
                    // std::cout << file_path << "\nAdd pose:\n"
                    //           << pose.toStyledString() << std::endl;
                    std::cout << file_path << "\nAdd pose:\n"
                            << pose["name"] << std::endl;
                }
                setJPDataModifiedParam(true);
            }else{
                std::cout << file_path << "\njointcornerobtainabnormal:\n" << std::endl;
            }
            ifile.close();
            std::ofstream ofile(file_path, std::ios::binary);
            ofile << writer.write(root);
            ofile.close();
            return vaild_flag;
        }return false;
    }

    const std::string *RT_left_station_name_=nullptr,*RT_right_station_name_=nullptr;
    std::unordered_map<std::string, std::vector<double> > *RT_joints__=nullptr;
    std::unordered_map<std::string, geometry_msgs::Pose> *RT_abs_pose__=nullptr;
    std::unordered_map<std::string, int> *RT_station_id__=nullptr;

    /*------------------Save and load related functions------------------------------*/
    // fromrobot_arm.hChinesecopyAnd come here，Only to obtain independence from the robotic armofloadRecord functions
    void load_joint_pose(const std::string &station_name,
                         std::unordered_map<std::string, std::vector<double>> &joints_,
                         std::unordered_map<std::string, geometry_msgs::Pose> &abs_pose_,
                         std::unordered_map<std::string, int> &station_id_)
    {   
        if(RT_left_station_name_==nullptr){
            if(station_name.length()>=5&&station_name.substr(0,5)=="left_")
                RT_left_station_name_=&station_name;
            else if(station_name.length()>=6&&station_name.substr(0,6)=="right_")
                RT_right_station_name_=&station_name;
            RT_joints__=&joints_;
            RT_abs_pose__=&abs_pose_;
            RT_station_id__=&station_id_;
        }
        ROS_INFO(">>>>>>>>> station_name=%s",station_name.c_str());

        std::string file_root = ros::package::getPath("manipulator") + "/data/";
        Json::Reader jsonreader;
        Json::Value root;
        std::string file_path = file_root + station_name + ".json";
        std::ifstream file(file_path, std::ios::binary);
        if (jsonreader.parse(file, root))
        {
            // Obtain joint angles
            Json::Value value = root["joints"];
            std::string name;
            for (int i = 0; i < value.size(); ++i)
            {
                name = value[i]["name"].asString();
                joints_[name].resize(6);
                for (int j = 0; j < 6; ++j)
                    joints_[name].at(j) = value[i]["value"][j].asDouble();
            }
            // Obtain the relative label pose of the instrument
            value = root["poses"];
            geometry_msgs::Pose pose;
            for (int i = 0; i < value.size(); ++i)
            {
                name = value[i]["name"].asString();
                pose.position.x = value[i]["value"][0].asDouble();
                pose.position.y = value[i]["value"][1].asDouble();
                pose.position.z = value[i]["value"][2].asDouble();
                pose.orientation.x = value[i]["value"][3].asDouble();
                pose.orientation.y = value[i]["value"][4].asDouble();
                pose.orientation.z = value[i]["value"][5].asDouble();
                pose.orientation.w = value[i]["value"][6].asDouble();
                // rel_pose_[name] = pose;
                abs_pose_[name] = pose;
            }
            // obtainmarker_id
            value = root["marker"];
            station_id_[station_name] = value["id"].asInt();
        }
        setJPDataModifiedParam(false);
        file.close();
    }
    void reload_joint_pose(){
        if(getJPDataModifiedParam()&&RT_left_station_name_!=nullptr){
            load_joint_pose(*RT_left_station_name_, *RT_joints__, *RT_abs_pose__, *RT_station_id__);
            load_joint_pose(*RT_right_station_name_, *RT_joints__, *RT_abs_pose__, *RT_station_id__);
        }
    }
    bool get_joint(const std::string &name,
                    std::unordered_map<std::string, std::vector<double>> &joints_,
                    std::vector<double> & joints)
    {
        reload_joint_pose();
        auto it = joints_.find(name);
        if (it == joints_.end())
        {
            ROS_ERROR_STREAM("no_joints_value:" + name);
            return false;
        }
        joints=it->second;
        return true;
    }

    bool get_abs_pose(const std::string &name,
                    std::unordered_map<std::string, geometry_msgs::Pose> &abs_pose_,
                    geometry_msgs::Pose& pose)
    {
        reload_joint_pose();
        auto it = abs_pose_.find(name);
        if (it == abs_pose_.end())
        {
            ROS_ERROR_STREAM("no_absolute_pose_value:" + name);
            return false;
        }
        pose=it->second;
        return true;
    }

    bool get_rel_pose(const std::string &name,
                    std::unordered_map<std::string, geometry_msgs::Pose> &rel_pose_,
                    geometry_msgs::Pose& pose)
    {
        reload_joint_pose();
        auto it = rel_pose_.find(name);
        if (it == rel_pose_.end())
        {
            ROS_ERROR_STREAM("no_absolute_pose_value:" + name);
            return false;
        }
        pose=it->second;
        return true;
    }
    /*------------------Save and load related functions------------------------------*/

    /*------------------saveloadtrajectoryfunction(yaml)------------------------------*/
    // Translate into English RobotTrajectory Serialize as YAML
    YAML::Node convertTrajectoryToYAML(const moveit_msgs::RobotTrajectory& trajectory)
    {
        YAML::Node node;

        for (const auto& joint_trajectory_point : trajectory.joint_trajectory.points)
        {
            YAML::Node point;
            point["positions"] = joint_trajectory_point.positions;
            point["velocities"] = joint_trajectory_point.velocities;
            point["accelerations"] = joint_trajectory_point.accelerations;
            point["effort"] = joint_trajectory_point.effort;
            point["time_from_start"] = joint_trajectory_point.time_from_start.toSec();
            node["joint_trajectory"]["points"].push_back(point);
        }
        node["joint_trajectory"]["joint_names"] = trajectory.joint_trajectory.joint_names;

        return node;
    }

    // save YAML document
    void saveTrajectoryToYAML(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename)
    {
        YAML::Node node = convertTrajectoryToYAML(trajectory);

        std::string file_root =
            ros::package::getPath("manipulator") + "/data/yaml_data/";
        std::string use_filename = file_root + filename;

        std::ofstream fout(use_filename);
        fout << node;
        fout.close();
    }

    // Translate into English YAML convert to RobotTrajectory
    moveit_msgs::RobotTrajectory convertYAMLToTrajectory(const YAML::Node& node)
    {
        moveit_msgs::RobotTrajectory trajectory;

        const YAML::Node& joint_trajectory_node = node["joint_trajectory"];
        trajectory.joint_trajectory.joint_names = joint_trajectory_node["joint_names"].as<std::vector<std::string>>();

        for (const auto& point_node : joint_trajectory_node["points"])
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = point_node["positions"].as<std::vector<double>>();
            point.velocities = point_node["velocities"].as<std::vector<double>>();
            point.accelerations = point_node["accelerations"].as<std::vector<double>>();
            point.effort = point_node["effort"].as<std::vector<double>>();
            point.time_from_start = ros::Duration(point_node["time_from_start"].as<double>());
            trajectory.joint_trajectory.points.push_back(point);
        }

        return trajectory;
    }

    // from YAML documentloadtrajectory
    void loadTrajectoryFromYAML(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename)
    {
        std::string file_root =
            ros::package::getPath("manipulator") + "/data/yaml_data/";
        std::string use_filename = file_root + filename;

        YAML::Node node = YAML::LoadFile(use_filename);
        trajectory = convertYAMLToTrajectory(node);
    }
    /*------------------saveloadtrajectoryfunction(yaml)------------------------------*/



}

namespace Operate_tool
{
    // Program blocking，convenientaPeople choose
    int interrupt_judge(bool enable_flag)
    {
        using namespace std;
        setlocale(LC_ALL, "");

        if (!enable_flag)
            return 0;
        else
        {
            cout << "Are you sure to proceed to the next step,Please enterY/N:" << endl;
            char judge_char;
            int return_num;
            bool break_flag = 0;
            while (!break_flag)
            {
                cin >> judge_char;

                switch (judge_char)
                {
                case 'Y':
                    cout << "Confirmed，Next steps will be taken......" << endl;
                    return_num = 1;
                    break_flag = 1;
                    break;
                case 'N':
                    cout << "consider, deliberate, weighonedownagainenterlineoperationof......" << endl;
                    return_num = 0;
                    break_flag = 1;
                    break;
                default:
                    cout << "Input error，Please re-enter！" << endl;
                    return_num = 0;
                    break_flag = 0;
                    break;
                }
                // enterlinebuffer，Eliminate carriage return
                cin.get();
            }
            return return_num;
        }
    }

    bool switch_controller(const std::string start_controller,
                           const std::string end_controller,
                           std::string switch_goalrobot)
    {
        controller_manager_msgs::SwitchController switchRequest;
        ros::ServiceClient switchClient;
        ros::NodeHandle nh;

        std::string switch_controller_name;
        switch_controller_name = switch_goalrobot + "/controller_manager/switch_controller";
        switchClient = nh.serviceClient<controller_manager_msgs::SwitchController>(switch_controller_name);
        // Fill in the request with the controllers you want to start and stop
        switchRequest.request.start_controllers.push_back(start_controller);
        switchRequest.request.stop_controllers.push_back(end_controller);

        // Indicate whether you want to strict or best-effort switching
        switchRequest.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

        // Call the service to switch controllers
        bool finnish_flag = switchClient.call(switchRequest);

        if (finnish_flag)
        {
            if (switchRequest.response.ok)
            {
                ROS_INFO("Controller switch was successful.");
            }
            else
            {
                ROS_ERROR("Controller switch failed");
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call controller switch service.");
            return false;
        }
        return true;
    }

    /*-----------------------End torque conversion-----------------------------------------------*/
    FT_transition::FT_transition(ros::NodeHandle &nh): tfBuffer_(), tfListener_(tfBuffer_)
    {
        ROS_INFO_NAMED("FT_transition", "FT_transition: FT topic is to open .....");

        nh_ = nh;
        ft_wrench_sub_right = nh_.subscribe("/right_robot/wrench", 10, &FT_transition::right_FT_wrenchCallback, this);
        ft_wrench_pub_right = nh_.advertise<geometry_msgs::WrenchStamped>("/right_robot/ur5e_current_wrench", 10);

        ft_wrench_sub_left = nh_.subscribe("/left_robot/wrench", 10, &FT_transition::left_FT_wrenchCallback, this);
        ft_wrench_pub_left = nh_.advertise<geometry_msgs::WrenchStamped>("/left_robot/ur5e_current_wrench", 10);

        right_force_modulus_pub = nh.advertise<std_msgs::Float64>("/right_force_modulus", 10);
        left_force_modulus_pub = nh.advertise<std_msgs::Float64>("/left_force_modulus", 10);
        right_torque_modulus_pub = nh.advertise<std_msgs::Float64>("/right_torque_modulus", 10);
        left_torque_modulus_pub = nh.advertise<std_msgs::Float64>("/left_torque_modulus", 10);
        left_force_x_pub = nh.advertise<std_msgs::Float64>("/left_force_x", 10);
        left_force_y_pub = nh.advertise<std_msgs::Float64>("/left_force_y", 10);
        left_force_z_pub = nh.advertise<std_msgs::Float64>("/left_force_z", 10);
        right_torque_x_pub= nh.advertise<std_msgs::Float64>("/right_torque_x", 10);
        right_torque_y_pub= nh.advertise<std_msgs::Float64>("/right_torque_y", 10);
        right_torque_z_pub= nh.advertise<std_msgs::Float64>("/right_torque_z", 10);
        right_torque_z_max_pub= nh.advertise<std_msgs::Float64>("/right_torque_z_max", 10);


        // createCoordinate conversion monitoringwait...
        // readaheadFT_flagparameter
        if (nh.getParam("A_manipulator/aheadFT_flag", aheadFT_flag))
            ROS_INFO("Got param: %d", aheadFT_flag);
        else
            ROS_ERROR("Failed to get param 'aheadFT_flag'");
        ros::Duration(0.5).sleep();
        ROS_INFO_NAMED("FT_transition", "FT_transition: FT topic has been opened!");

    }

    FT_transition::~FT_transition()
    {
        ROS_INFO_NAMED("FT_transition", "FT_transition: This Class is to free.....");
        // Need to close subscription topic，to avoidpubdestroyed ，But Ren Zhilinerelease
        ft_wrench_sub_right.shutdown();
        ft_wrench_sub_left.shutdown();
    }

    void FT_transition::coordinate_transFT(geometry_msgs::WrenchStamped &msg)
    {
        // //////////////----keep onlyzaxisdata------//////////////
        // msg.wrench.force.x = 0;
        // msg.wrench.force.y = 0;
        // msg.wrench.force.z = 
        //     abs(msg.wrench.force.z) < 1 ? 0 : msg.wrench.force.z;
        // msg.wrench.torque.x = 0;
        // msg.wrench.torque.y = 0;
        // msg.wrench.torque.z = 0;      
        /////////////////////////////////////////////////
        wTot_tfStamped = tfBuffer_.lookupTransform("world", msg.header.frame_id, ros::Time(0));
        tf2::Quaternion tf_quat(wTot_tfStamped.transform.rotation.x,
                                wTot_tfStamped.transform.rotation.y,
                                wTot_tfStamped.transform.rotation.z,
                                wTot_tfStamped.transform.rotation.w);
        tf2::Matrix3x3 tf_rotation(tf_quat);
        Eigen::Matrix3d rotation_Matrix;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rotation_Matrix(i, j) = tf_rotation[i][j];

        Eigen::Vector3d force(msg.wrench.force.x, 
                              msg.wrench.force.y, 
                              msg.wrench.force.z);
        Eigen::Vector3d torque(msg.wrench.torque.x, 
                               msg.wrench.torque.y, 
                               msg.wrench.torque.z);

        force = rotation_Matrix * force;
        torque = rotation_Matrix * torque;

        // Translate into EnglishResult reversal messageclasstype
        msg.wrench.force.x = force[0];
        msg.wrench.force.y = force[1];
        msg.wrench.force.z = force[2];

        msg.wrench.torque.x = torque[0];
        msg.wrench.torque.y = torque[1];
        msg.wrench.torque.z = torque[2];
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
    }

    /*--------------------------B1.Torque conversion callback function---------------------------*/
    void FT_transition::right_FT_wrenchCallback(geometry_msgs::WrenchStamped msg)
    {
        msg.header.frame_id = "right_tool0";
        msg.header.stamp = ros::Time::now();
        // if(aheadFT_flag)coordinate_transFT(msg);//Convert force data to world coordinates

        ft_wrench_pub_right.publish(msg);

        std_msgs::Float64 msg_data_pub;
        msg_data_pub.data = std::sqrt(msg.wrench.force.x * msg.wrench.force.x + 
                                      msg.wrench.force.y * msg.wrench.force.y +
                                      msg.wrench.force.z * msg.wrench.force.z);
        right_force_modulus_pub.publish(msg_data_pub);

        msg_data_pub.data = std::sqrt(msg.wrench.torque.x * msg.wrench.torque.x + 
                                      msg.wrench.torque.y * msg.wrench.torque.y +
                                      msg.wrench.torque.z * msg.wrench.torque.z);
        right_torque_modulus_pub.publish(msg_data_pub);

        msg_data_pub.data=right_torque_x_filter(msg.wrench.torque.x);
        right_torque_x_pub.publish(msg_data_pub);
        msg_data_pub.data=right_torque_y_filter(msg.wrench.torque.y);
        right_torque_y_pub.publish(msg_data_pub);
        msg_data_pub.data=right_torque_z_filter(msg.wrench.torque.z);
        right_torque_z_pub.publish(msg_data_pub);

        msg_data_pub.data=right_torque_z_maxer(msg.wrench.torque.z);
        right_torque_z_max_pub.publish(msg_data_pub);
    }
    void FT_transition::left_FT_wrenchCallback(geometry_msgs::WrenchStamped msg)
    {
        msg.header.frame_id = "left_tool0";
        msg.header.stamp = ros::Time::now();
        if(aheadFT_flag)coordinate_transFT(msg);//Convert force data to world coordinates

        ft_wrench_pub_left.publish(msg);

        std_msgs::Float64 msg_data_pub;
        msg_data_pub.data = std::sqrt(msg.wrench.force.x * msg.wrench.force.x + 
                                      msg.wrench.force.y * msg.wrench.force.y +
                                      msg.wrench.force.z * msg.wrench.force.z);
        left_force_modulus_pub.publish(msg_data_pub);

        msg_data_pub.data = std::sqrt(msg.wrench.torque.x * msg.wrench.torque.x + 
                                      msg.wrench.torque.y * msg.wrench.torque.y +
                                      msg.wrench.torque.z * msg.wrench.torque.z);
        left_torque_modulus_pub.publish(msg_data_pub);

        msg_data_pub.data=left_force_x_filter(msg.wrench.force.x);
        left_force_x_pub.publish(msg_data_pub);
        msg_data_pub.data=left_force_y_filter(msg.wrench.force.y);
        left_force_y_pub.publish(msg_data_pub);
        msg_data_pub.data=left_force_z_filter(msg.wrench.force.z);
        left_force_z_pub.publish(msg_data_pub);
    }
    
    robotiq_FT_transition::robotiq_FT_transition(ros::NodeHandle &nh, std::string robot_name, double* length_intool)
        : tfBuffer_(), tfListener_(tfBuffer_)
    {
        ROS_INFO_NAMED("robotiq_FT_transition", "robotiq_FT_transition: FT topic is to open .....");
        fixed_robot = robot_name;

        nh_ = nh;
        torque_usedMatrix << 0, -1 * length_intool[2], length_intool[1], 
                             length_intool[2], 0, -1 * length_intool[0],
                             -1 * length_intool[1], length_intool[0], 0;
        ///////////////////---------Load data---------/////////////////////////
        // load YAML document
        std::string file_root =
            ros::package::getPath("manipulator") + "/scripts/corrector_data/";
        std::string filename = "robotiq_fttf_result.yaml";
        std::string use_filename = file_root + filename;
        YAML::Node config = YAML::LoadFile(use_filename);
        // check fF_result Does the node exist
        if (!config["fF_result"])
        {
            std::cerr << "Error: fF_result not found in YAML file." << std::endl;
        }
        // extract F and f of x, y, z And assign it to Eigen::Vector3d
        F << config["fF_result"]["F"]["x"].as<double>(),
            config["fF_result"]["F"]["y"].as<double>(),
            config["fF_result"]["F"]["z"].as<double>();

        f << config["fF_result"]["f"]["x"].as<double>(),
            config["fF_result"]["f"]["y"].as<double>(),
            config["fF_result"]["f"]["z"].as<double>();
        ///////////////////---------Load data---------/////////////////////////

        ROS_INFO_NAMED("robotiq_FT_transition", "robotiq_FT_transition: Wait to tfBuffer_ .....");
        ros::Duration(0.5).sleep();
        robotiq_ft_wrench_sub = nh_.subscribe("/robotiq_ft_wrench", 10, &robotiq_FT_transition::robotiq_FT_wrenchCallback, this);
        robotiq_ft_wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>("/" + fixed_robot + "/robotiq_current_wrench", 10);
        ROS_INFO_NAMED("robotiq_FT_transition", "robotiq_FT_transition: tfBuffer_ have OK.");
    }

    robotiq_FT_transition::~robotiq_FT_transition()
    {
        ROS_INFO_NAMED("robotiq_FT_transition", "robotiq_FT_transition: This Class is to free.....");
        // Need to close subscription topic，to avoidpubdestroyed ，But Ren Zhilinerelease
        robotiq_ft_wrench_sub.shutdown();
    }

    void robotiq_FT_transition::robotiq_FT_wrenchCallback(const geometry_msgs::WrenchStamped &msg)
    {

        geometry_msgs::WrenchStamped ft_std_msg;
        // 1.Obtain torque data
        ft_std_msg.wrench = msg.wrench;
        Gravity_correction(ft_std_msg);

        // 2.Obtain torque head
        if (fixed_robot == "right_robot")
            ft_std_msg.header.frame_id = "right_tool0";
        else if (fixed_robot == "left_robot")
            ft_std_msg.header.frame_id = "left_tool0";
        else
            ft_std_msg.header.frame_id = "tool0";
        ft_std_msg.header.stamp = ros::Time::now();

        robotiq_ft_wrench_pub.publish(ft_std_msg);
    }

    void robotiq_FT_transition::Gravity_correction(geometry_msgs::WrenchStamped &msg)
    {
        wTot_tfStamped = tfBuffer_.lookupTransform("right_tool0", "world", ros::Time(0));
        tf2::Quaternion tf_quat(wTot_tfStamped.transform.rotation.x,
                                wTot_tfStamped.transform.rotation.y,
                                wTot_tfStamped.transform.rotation.z,
                                wTot_tfStamped.transform.rotation.w);
        tf2::Matrix3x3 tf_rotation(tf_quat);
        Eigen::Matrix3d rotation_Matrix;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation_Matrix(i, j) = tf_rotation[i][j];
            }
        }

        Eigen::Vector3d Gravity_vector; 
        Eigen::Vector3d Touch_force, Touch_torque;
        Gravity_vector = rotation_Matrix * f + F;

        Touch_force[0] = msg.wrench.force.x - Gravity_vector[0];
        Touch_force[1] = msg.wrench.force.y - Gravity_vector[1];
        Touch_force[2] = msg.wrench.force.z - Gravity_vector[2];

        Touch_torque = torque_usedMatrix * Touch_force;

        msg.wrench.force.x = Touch_force[0];
        msg.wrench.force.y = Touch_force[1];
        msg.wrench.force.z = Touch_force[2];
        msg.wrench.torque.x = Touch_torque[0];
        msg.wrench.torque.y = Touch_torque[1];
        msg.wrench.torque.z = Touch_torque[2];
    }

    void correct_coordinate(ros::NodeHandle nh, std::string init_coordinate, std::string ref_coordinate, tf::Matrix3x3 &matrix)
    {

        tf::TransformListener listener;
        tf::StampedTransform transform;

        bool display_flag = false;
        while (nh.ok())
        {
            if (listener.canTransform(init_coordinate, ref_coordinate, ros::Time(0)))
            {
                listener.lookupTransform(init_coordinate, ref_coordinate, ros::Time(0), transform);
                tf::Quaternion orientation = transform.inverse().getRotation();
                matrix.setRotation(orientation);

                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);
                // ROS_INFO("Have obtain Matrix, Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
                break;
            }
            else if (!display_flag)
            {
                display_flag = true;
                // ROS_WARN("Wait for listener response......");
            }
        }
    }

    geometry_msgs::Pose tool0_usePose(double length[3], geometry_msgs::Pose ref_Pose, bool dir_flag)
    {
        int dir_value;
        if (dir_flag)
            dir_value = 1;
        else
            dir_value = -1;

        tf::Quaternion quaternion;
        quaternion.setX(ref_Pose.orientation.x);
        quaternion.setY(ref_Pose.orientation.y);
        quaternion.setZ(ref_Pose.orientation.z);
        quaternion.setW(ref_Pose.orientation.w);

        // Obtain the transformation matrix
        tf::Matrix3x3 matrix(quaternion);

        // obtaintoolThe coordinates areworldUnder the coordinate systemofportion
        tf::Vector3 tool_size_intool0(length[0], length[1], length[2]);
        tf::Vector3 tool_size_inworld = tool_size_intool0 * matrix.inverse();

        geometry_msgs::Pose goal_Pose;
        goal_Pose.orientation = ref_Pose.orientation;
        goal_Pose.position.x = ref_Pose.position.x - dir_value * tool_size_inworld[0];
        goal_Pose.position.y = ref_Pose.position.y - dir_value * tool_size_inworld[1];
        goal_Pose.position.z = ref_Pose.position.z - dir_value * tool_size_inworld[2];

        return goal_Pose;
    }

    // Translate into EnglishOMPLPath conversion toMoveIt!pathofHelp function
    std::vector<robot_state::RobotStatePtr> omplPathToMoveItPath(const ompl::geometric::PathGeometric &path, 
                                                                 moveit::planning_interface::MoveGroupInterfacePtr mgptr)
    {
        std::vector<robot_state::RobotStatePtr> moveit_path;

        // Obtain joint group
        const robot_state::JointModelGroup *joint_model_group = mgptr->getRobotModel()->getJointModelGroup(mgptr->getName()); // replace"arm"Name your robotic arm group

        // traverseOMPLpathChineseofeachastatus
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const ompl::base::State *ompl_state = path.getState(i);

            // Translate into EnglishOMPLstatusconvert toMoveIt!status
            std::vector<double> joint_values;
            path.getSpaceInformation()->getStateSpace()->copyToReals(joint_values, ompl_state);

            // createoneanewofRobotState
            robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(mgptr->getRobotModel()));
            kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
            kinematic_state->update(); // morenewstatusTo ensure correctnessofforward kinematics 

            // Translate into EnglishRobotStateaddplustopathChinese
            moveit_path.push_back(kinematic_state);
        }

        return moveit_path;
    }

    std::vector<robot_state::RobotStatePtr> PathToMoveItPath(const moveit::planning_interface::MoveGroupInterface::Plan &path,
                                                             moveit::planning_interface::MoveGroupInterfacePtr mgptr)
    {
        std::vector<robot_state::RobotStatePtr> moveit_path;

        // Obtain joint group
        const robot_state::JointModelGroup *joint_model_group = mgptr->getCurrentState()->getJointModelGroup(mgptr->getName());

        // traverseMoveIt!pathChineseofeachatrajectorypoint
        for (const auto &trajectory_point : path.trajectory_.joint_trajectory.points)
        {
            // createoneanewofRobotState
            robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(*mgptr->getCurrentState()));
            
            // Translate into EnglishJoint valuefromtrajectorypointSettingstoRobotStateChinese
            kinematic_state->setJointGroupPositions(joint_model_group, trajectory_point.positions);
            kinematic_state->update(); // morenewstatusTo ensure correctnessofforward kinematics 

            // Translate into EnglishRobotStateaddplustopathChinese
            moveit_path.push_back(kinematic_state);
        }

        return moveit_path;
    }

    void Merge_trajectory(std::vector<robot_state::RobotStatePtr> &moveit_path,
                          std::vector<robot_state::RobotStatePtr> merged_path,
                          int startpath_point)
    {
        if (!moveit_path.empty())
        {
            // Use insert Translate into English moveit_path2 ofrest（Remove the first oneaelement）Insert into moveit_path ofend
            moveit_path.insert(moveit_path.end(), merged_path.begin() + startpath_point, merged_path.end());
    
        }
        else
        {
            moveit_path = merged_path;
        }
    }
     
    void Smooth_trajectory(moveit_msgs::RobotTrajectory &trajectory, double vel_acc_value,
                          std::vector<robot_state::RobotStatePtr> moveit_path,
                          moveit::planning_interface::MoveGroupInterfacePtr mgptr)
    {
        // 4.createMoveIt!trajectory
        robot_trajectory::RobotTrajectory robot_traj(mgptr->getRobotModel(), mgptr->getName());
        // 5.pairtrajectoryenterlinetimeinterpolation
        for (const auto &robot_state : moveit_path)
        {
            robot_traj.addSuffixWayPoint(robot_state, 0.0); // 0.02s time interval
        }
        // 6.Use IterativeParabolicTimeParameterization enterlinetimeparametertransformandsmoothing
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(robot_traj, vel_acc_value, vel_acc_value))
            std::cout << "Smooth_trajectory: Trajectory smoothed successfully." << std::endl;
        else
            std::cout << "Smooth_trajectory: Failed to smooth trajectory!" << std::endl;

        // 7.Obtain trajectory
        robot_traj.getRobotTrajectoryMsg(trajectory);        
    }

    void removeDuplicateTimeStamps(moveit_msgs::RobotTrajectory& trajectory)
    {
        auto& points = trajectory.joint_trajectory.points;
        // If the number of path points is less than2a，No need to go heavy
        if (points.size() < 2)
            return;
        // traversepathpoint，deletetimestampsameofpoint
        for (auto it = points.begin(); it != points.end() - 1; )
        {
            auto next_it = std::next(it);
            // checkAdjacent TwoapointoftimestampyesNo, they are the same
            if (it->time_from_start == next_it->time_from_start)
            {// Remove duplicatesofpoint
                it = points.erase(next_it); 
            }
            else
            {// Only when the timestamps are different，Just movetodownoneapoint
                ++it;
            }
        }
    }

    geometry_msgs::Pose Get_LtargetPose(geometry_msgs::Pose r_target_pose,
                                        geometry_msgs::Pose l_initpose,
                                        geometry_msgs::Pose r_initpose)
    {
        geometry_msgs::Pose l_init_pose, r_init_pose;
        l_init_pose = l_initpose;
        r_init_pose = r_initpose;

        Eigen::MatrixXd r_init_oRE2_matrix(3, 3), r_target_oRE2_matrix(3, 3);
        tf::Quaternion r_init_quaternion, r_target_quaternion;
        tf::quaternionMsgToTF(r_init_pose.orientation, r_init_quaternion);
        tf::quaternionMsgToTF(r_target_pose.orientation, r_target_quaternion);
        tf::Matrix3x3 r_init_R_matrix(r_init_quaternion);     // createonea3x3ofrotation matrix
        tf::Matrix3x3 r_target_R_matrix(r_target_quaternion); // createonea3x3ofrotation matrix
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
            {
                r_init_oRE2_matrix(i, j) = r_init_R_matrix[i][j];
                r_target_oRE2_matrix(i, j) = r_target_R_matrix[i][j];
            }

        Eigen::Vector3d r_postion, l_postion, realtive_temp_LtoR; // 3*1
        r_postion << r_target_pose.position.x,
                     r_target_pose.position.y,
                     r_target_pose.position.z;
        realtive_temp_LtoR << (l_init_pose.position.x - r_init_pose.position.x),
                            (l_init_pose.position.y - r_init_pose.position.y),
                            (l_init_pose.position.z - r_init_pose.position.z);
        l_postion = r_postion + r_target_oRE2_matrix * r_init_oRE2_matrix.transpose() * realtive_temp_LtoR;

        Eigen::Vector4d right_wxyz_E2;
        Eigen::Vector4d wxyz_init_LtoR;
        Eigen::Vector4d left_init_wxyz_E1_inverse, right_init_wxyz_E2;

        right_wxyz_E2 << r_target_pose.orientation.w,
                        r_target_pose.orientation.x,
                        r_target_pose.orientation.y,
                        r_target_pose.orientation.z;
        left_init_wxyz_E1_inverse << l_init_pose.orientation.w,
                                -1 * l_init_pose.orientation.x,
                                -1 * l_init_pose.orientation.y,
                                -1 * l_init_pose.orientation.z;
        right_init_wxyz_E2 << r_init_pose.orientation.w,
                            r_init_pose.orientation.x,
                            r_init_pose.orientation.y,
                            r_init_pose.orientation.z;
        // $t^-1
        wxyz_init_LtoR = Constraint_tools::Constraint_M(left_init_wxyz_E1_inverse, true) * right_init_wxyz_E2;
        for (size_t i = 1; i < 4; i++)
            wxyz_init_LtoR(i) = -1 * wxyz_init_LtoR(i);
        Eigen::MatrixXd M_temp_foq;
        M_temp_foq = Constraint_tools::Constraint_M(wxyz_init_LtoR, true) *
                    Constraint_tools::Constraint_M(right_wxyz_E2, false);
        Eigen::Vector4d temp_unit_q;
        temp_unit_q << 1, 0, 0, 0;
        Eigen::Vector4d left_wxyz_E1_inverse, left_wxyz_E1;
        left_wxyz_E1_inverse = M_temp_foq.transpose() * temp_unit_q;
        for (size_t i = 1; i < 4; i++)
            left_wxyz_E1(i) = -1 * left_wxyz_E1_inverse(i);
        left_wxyz_E1(0) = sqrt(1 - pow(left_wxyz_E1(1), 2) - pow(left_wxyz_E1(2), 2) - pow(left_wxyz_E1(3), 2));
        
        geometry_msgs::Pose l_target_pose;
        l_target_pose.position.x = l_postion(0);
        l_target_pose.position.y = l_postion(1);
        l_target_pose.position.z = l_postion(2);
        l_target_pose.orientation.w = left_wxyz_E1(0);
        l_target_pose.orientation.x = left_wxyz_E1(1);
        l_target_pose.orientation.y = left_wxyz_E1(2);
        l_target_pose.orientation.z = left_wxyz_E1(3);   

        return l_target_pose; 
    }
}

namespace Motion_tool
{
    // Using arcsdegreemanufacture
    double spiral_motion_tool(moveit::planning_interface::MoveGroupInterfacePtr mgtr,
                              double screw_pitch, double expected_angle, bool motor_direction,
                              bool execute_flag)
    {
        int motor_direction_value;
        // Twisting direction（Counterclockwise and clockwise，Clockwise negative）
        if (!motor_direction)
            motor_direction_value = -1;
        else
            motor_direction_value = 1;

        mgtr->setStartStateToCurrentState();

        // When planning，Maximum rotation per time180degree
        double revolutions = 0.5; // Number of spiral turns
        int total_points = 100;   // Spiral Lineoftotalpointnumber
        double surplus_angle;     // remainingcornerdegree
        if (expected_angle >= M_PI)
        {
            revolutions = 0.5;
            total_points = 100;
            surplus_angle = expected_angle - M_PI;
        }
        else
        {
            revolutions = 0.5 * expected_angle / M_PI;
            total_points = (int)(100 * expected_angle / M_PI);
            surplus_angle = 0.0;
        }

        // obtainCurrent End Executionlinedeviceofpose
        geometry_msgs::PoseStamped current_pose = mgtr->getCurrentPose();
        // obtaincurrentrobotic armendofposture（At this point, the gripper has already grasped the bottle cap）
        tf2::Quaternion orientation;
        tf2::fromMsg(current_pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Generate a spiral path
        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 0; i < total_points; ++i)
        {
            double d_angle = motor_direction_value * 2.0 * M_PI * revolutions * ((double)(i + 1) / (double)total_points);
            double temp_roll, temp_pitch, temp_yaw;
            // modifyZaxisofrotation angledegreeforcurrentcornerdegreeplus30degree
            temp_roll = roll;
            temp_pitch = pitch;
            temp_yaw = yaw - d_angle; // windZaxis rotation30degree

            // Translate into EnglishmodifyafterofEuler anglesconvert toquaternion
            orientation.setRPY(temp_roll, temp_pitch, temp_yaw);

            // Translate into EnglishnewofEnd of pipe executionlinedevicepostureSettingsFor the target posture
            current_pose.pose.orientation = tf2::toMsg(orientation);
            geometry_msgs::Pose next_pose = current_pose.pose;
            next_pose.position.z = current_pose.pose.position.z - (motor_direction_value * d_angle * screw_pitch);

            waypoints.push_back(next_pose);
        }

        // Convert the path into joint space
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = mgtr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // Settingspathpointoftimestamp
        double time_from_start = 0.0;
        ros::Time start_time = ros::Time::now();
        for (int i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
        {
            trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(time_from_start);
            time_from_start += 0.05; // Settingstime intervalfor0.1second
        }

        // Execution path
        mgtr->execute(trajectory);

        ros::Duration(0.5).sleep();

        return surplus_angle;
    }

    // void GetStretch_motion_tool(moveit::planning_interface::MoveGroupInterfacePtr mgtr,
    //                             geometry_msgs::Pose left_ref_Pose, double left_length,
    //                             geometry_msgs::Pose right_ref_Pose, double right_length,
    //                             bool execute_flag)
    // {
    //     mgtr->setStartStateToCurrentState();
    //     // Generate stretching rules
    //     std::vector<geometry_msgs::Pose> waypoints;

    // }

    void twist_motion_single(ros::NodeHandle nh, std::string robot_name,
                             double speed_scale, double change_value,
                             TwistMove twisttype,
                             std::vector<double> RPY_correct, std::string ref_coordinate)
    {
        std::string twist_command_name;
        twist_command_name = "/" + robot_name + "/twist_controller/command";

        std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> realtime_pub;
        realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, twist_command_name, 4);

        // obtain **_base--------------
        std::string delimiter = "_", init_coordinate;
        std::string extracted_string;

        size_t pos = robot_name.find(delimiter);
        if (pos != std::string::npos)
        {
            extracted_string = robot_name.substr(0, pos);
        }
        init_coordinate = "/" + extracted_string + "_base";
        // obtain **_base--------------

        // Obtain the transformation matrix
        tf::Matrix3x3 matrix, init_matrix, RPY_matrix;
        Operate_tool::correct_coordinate(nh, init_coordinate, ref_coordinate, init_matrix);
        RPY_matrix.setRPY(RPY_correct[0], RPY_correct[1], RPY_correct[2]);
        matrix = RPY_matrix.inverse() * init_matrix;

        Operate_tool::switch_controller("twist_controller",
                                        "scaled_pos_joint_traj_controller", robot_name);

        // delay，Waiting for the above preparations to be complete
        ros::Duration(0.5).sleep();

        double temp_values[6] = {0, 0, 0, 0, 0, 0};
        switch (twisttype)
        {
        case lINEAR_X:
            temp_values[0] = (change_value / fabs(change_value)) * speed_scale;
            break;
        case lINEAR_Y:
            temp_values[1] = (change_value / fabs(change_value)) * speed_scale;
            break;
        case lINEAR_Z:
            temp_values[2] = (change_value / fabs(change_value)) * speed_scale;
            break;
        case ANGULAR_X:
            temp_values[3] = (change_value / fabs(change_value)) * speed_scale;
            break;
        case ANGULAR_Y:
            temp_values[4] = (change_value / fabs(change_value)) * speed_scale;
            break;
        case ANGULAR_Z:
            temp_values[5] = (change_value / fabs(change_value)) * speed_scale;
            break;
        }
        tf::Vector3 world_linear_velocity(temp_values[0], temp_values[1], temp_values[2]);
        tf::Vector3 world_angular_velocity(temp_values[3], temp_values[4], temp_values[5]);

        // std::cout<<world_linear_velocity[0]<<" "<<world_linear_velocity[1]<<" "<<world_linear_velocity[2]<<std::endl;

        tf::Vector3 right_base_linear_velocity = world_linear_velocity * matrix;
        tf::Vector3 right_base_angular_velocity = world_angular_velocity * matrix;

        // std::cout<<right_base_linear_velocity[0]<<" "<<right_base_linear_velocity[1]<<" "<<right_base_linear_velocity[2]<<std::endl;

        realtime_pub->msg_.linear.x = right_base_linear_velocity.getX();
        realtime_pub->msg_.linear.y = right_base_linear_velocity.getY();
        realtime_pub->msg_.linear.z = right_base_linear_velocity.getZ();
        realtime_pub->msg_.angular.x = right_base_angular_velocity.getX();
        realtime_pub->msg_.angular.y = right_base_angular_velocity.getY();
        realtime_pub->msg_.angular.z = right_base_angular_velocity.getZ();
        realtime_pub->unlockAndPublish();

        std::cout << "--------wait---------" << std::endl;
        double run_time = fabs(1.0 * change_value / speed_scale);
        ros::Duration(run_time).sleep();

        realtime_pub->msg_.linear.x = 0;
        realtime_pub->msg_.linear.y = 0;
        realtime_pub->msg_.linear.z = 0;
        realtime_pub->msg_.angular.x = 0;
        realtime_pub->msg_.angular.y = 0;
        realtime_pub->msg_.angular.z = 0;
        realtime_pub->unlockAndPublish();

        Operate_tool::switch_controller("scaled_pos_joint_traj_controller",
                                        "twist_controller", robot_name);
    }

    void twist_motion_dual(ros::NodeHandle nh, std::string left_robot_name, std::string right_robot_name,
                           double left_speed_scale, double right_speed_scale,
                           double left_change_value, double right_change_value,
                           TwistMove left_twisttype, TwistMove right_twisttype,
                           std::vector<double> RPY_correct, std::string ref_coordinate)
    {
        std::string left_twist_command_name, right_twist_command_name;
        left_twist_command_name = "/" + left_robot_name + "/twist_controller/command";
        right_twist_command_name = "/" + right_robot_name + "/twist_controller/command";

        std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> left_realtime_pub, right_realtime_pub;
        left_realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, left_twist_command_name, 4);
        right_realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, right_twist_command_name, 4);

        // obtain **_base--------------
        std::string delimiter = "_", left_init_coordinate, right_init_coordinate;
        std::string left_extracted_string, right_extracted_string;

        size_t left_pos = left_robot_name.find(delimiter);
        if (left_pos != std::string::npos)
        {
            left_extracted_string = left_robot_name.substr(0, left_pos);
        }
        size_t right_pos = right_robot_name.find(delimiter);
        if (right_pos != std::string::npos)
        {
            right_extracted_string = right_robot_name.substr(0, right_pos);
        }
        left_init_coordinate = "/" + left_extracted_string + "_base";
        right_init_coordinate = "/" + right_extracted_string + "_base";

        // Obtain the transformation matrix
        tf::Matrix3x3 left_matrix, right_matrix;
        tf::Matrix3x3 left_init_matrix, right_init_matrix, RPY_matrix;
        Operate_tool::correct_coordinate(nh, left_init_coordinate, ref_coordinate, left_init_matrix);
        Operate_tool::correct_coordinate(nh, right_init_coordinate, ref_coordinate, right_init_matrix);
        RPY_matrix.setRPY(RPY_correct[0], RPY_correct[1], RPY_correct[2]);
        left_matrix = RPY_matrix.inverse() * left_init_matrix;
        right_matrix = RPY_matrix.inverse() * right_init_matrix;

        Operate_tool::switch_controller("twist_controller",
                                        "scaled_pos_joint_traj_controller", left_robot_name);
        Operate_tool::switch_controller("twist_controller",
                                        "scaled_pos_joint_traj_controller", right_robot_name);

        // delay，Waiting for the above preparations to be complete
        ros::Duration(0.5).sleep();

        double left_temp_values[6] = {0, 0, 0, 0, 0, 0};
        switch (left_twisttype)
        {
        case lINEAR_X:
            left_temp_values[0] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        case lINEAR_Y:
            left_temp_values[1] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        case lINEAR_Z:
            left_temp_values[2] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        case ANGULAR_X:
            left_temp_values[3] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        case ANGULAR_Y:
            left_temp_values[4] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        case ANGULAR_Z:
            left_temp_values[5] = (left_change_value / fabs(left_change_value)) * left_speed_scale;
            break;
        }
        double right_temp_values[6] = {0, 0, 0, 0, 0, 0};
        switch (right_twisttype)
        {
        case lINEAR_X:
            right_temp_values[0] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        case lINEAR_Y:
            right_temp_values[1] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        case lINEAR_Z:
            right_temp_values[2] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        case ANGULAR_X:
            right_temp_values[3] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        case ANGULAR_Y:
            right_temp_values[4] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        case ANGULAR_Z:
            right_temp_values[5] = (right_change_value / fabs(right_change_value)) * right_speed_scale;
            break;
        }

        tf::Vector3 left_world_linear_velocity(left_temp_values[0], left_temp_values[1], left_temp_values[2]);
        tf::Vector3 left_world_angular_velocity(left_temp_values[3], left_temp_values[4], left_temp_values[5]);
        tf::Vector3 right_world_linear_velocity(right_temp_values[0], right_temp_values[1], right_temp_values[2]);
        tf::Vector3 right_world_angular_velocity(right_temp_values[3], right_temp_values[4], right_temp_values[5]);

        tf::Vector3 left_base_linear_velocity = left_world_linear_velocity * left_matrix;
        tf::Vector3 left_base_angular_velocity = left_world_angular_velocity * left_matrix;
        tf::Vector3 right_base_linear_velocity = right_world_linear_velocity * right_matrix;
        tf::Vector3 right_base_angular_velocity = right_world_angular_velocity * right_matrix;

        left_realtime_pub->msg_.linear.x = left_base_linear_velocity.getX();
        left_realtime_pub->msg_.linear.y = left_base_linear_velocity.getY();
        left_realtime_pub->msg_.linear.z = left_base_linear_velocity.getZ();
        left_realtime_pub->msg_.angular.x = left_base_angular_velocity.getX();
        left_realtime_pub->msg_.angular.y = left_base_angular_velocity.getY();
        left_realtime_pub->msg_.angular.z = left_base_angular_velocity.getZ();

        right_realtime_pub->msg_.linear.x = right_base_linear_velocity.getX();
        right_realtime_pub->msg_.linear.y = right_base_linear_velocity.getY();
        right_realtime_pub->msg_.linear.z = right_base_linear_velocity.getZ();
        right_realtime_pub->msg_.angular.x = right_base_angular_velocity.getX();
        right_realtime_pub->msg_.angular.y = right_base_angular_velocity.getY();
        right_realtime_pub->msg_.angular.z = right_base_angular_velocity.getZ();

        left_realtime_pub->unlockAndPublish();
        right_realtime_pub->unlockAndPublish();

        double left_run_time = fabs(1.0 * left_change_value / left_speed_scale);
        double right_run_time = fabs(1.0 * right_change_value / right_speed_scale);

        if (left_run_time != right_run_time)
        {
            if (left_run_time < right_run_time)
            {
                ros::Duration(left_run_time).sleep();
                left_realtime_pub->msg_.linear.x = 0;
                left_realtime_pub->msg_.linear.y = 0;
                left_realtime_pub->msg_.linear.z = 0;
                left_realtime_pub->msg_.angular.x = 0;
                left_realtime_pub->msg_.angular.y = 0;
                left_realtime_pub->msg_.angular.z = 0;
                left_realtime_pub->unlockAndPublish();
                ros::Duration(right_run_time - left_run_time).sleep();
                right_realtime_pub->msg_.linear.x = 0;
                right_realtime_pub->msg_.linear.y = 0;
                right_realtime_pub->msg_.linear.z = 0;
                right_realtime_pub->msg_.angular.x = 0;
                right_realtime_pub->msg_.angular.y = 0;
                right_realtime_pub->msg_.angular.z = 0;
                right_realtime_pub->unlockAndPublish();
            }
            else
            {
                ros::Duration(right_run_time).sleep();
                right_realtime_pub->msg_.linear.x = 0;
                right_realtime_pub->msg_.linear.y = 0;
                right_realtime_pub->msg_.linear.z = 0;
                right_realtime_pub->msg_.angular.x = 0;
                right_realtime_pub->msg_.angular.y = 0;
                right_realtime_pub->msg_.angular.z = 0;
                right_realtime_pub->unlockAndPublish();
                ros::Duration(left_run_time - right_run_time).sleep();
                left_realtime_pub->msg_.linear.x = 0;
                left_realtime_pub->msg_.linear.y = 0;
                left_realtime_pub->msg_.linear.z = 0;
                left_realtime_pub->msg_.angular.x = 0;
                left_realtime_pub->msg_.angular.y = 0;
                left_realtime_pub->msg_.angular.z = 0;
                left_realtime_pub->unlockAndPublish();
            }
        }
        else
        {
            ros::Duration(left_run_time).sleep();
            left_realtime_pub->msg_.linear.x = 0;
            left_realtime_pub->msg_.linear.y = 0;
            left_realtime_pub->msg_.linear.z = 0;
            left_realtime_pub->msg_.angular.x = 0;
            left_realtime_pub->msg_.angular.y = 0;
            left_realtime_pub->msg_.angular.z = 0;
            right_realtime_pub->msg_.linear.x = 0;
            right_realtime_pub->msg_.linear.y = 0;
            right_realtime_pub->msg_.linear.z = 0;
            right_realtime_pub->msg_.angular.x = 0;
            right_realtime_pub->msg_.angular.y = 0;
            right_realtime_pub->msg_.angular.z = 0;
            left_realtime_pub->unlockAndPublish();
            right_realtime_pub->unlockAndPublish();
        }

        Operate_tool::switch_controller("scaled_pos_joint_traj_controller",
                                        "twist_controller", left_robot_name);
        Operate_tool::switch_controller("scaled_pos_joint_traj_controller",
                                        "twist_controller", right_robot_name);
    }
    void twist_move(ros::NodeHandle nh, std::string robot_name, TwistMoveMsg msg,
                    double run_time, std::vector<double> RPY_correct, std::string ref_coordinate)
    {
        std::string twist_command_name;
        twist_command_name = "/" + robot_name + "/twist_controller/command";

        std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> realtime_pub;
        realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, twist_command_name, 4);

        // obtain **_base--------------
        std::string delimiter = "_", init_coordinate;
        std::string extracted_string;

        size_t pos = robot_name.find(delimiter);
        if (pos != std::string::npos)
        {
            extracted_string = robot_name.substr(0, pos);
        }
        init_coordinate = "/" + extracted_string + "_base";
        // obtain **_base--------------

        // Obtain the transformation matrix
        tf::Matrix3x3 matrix, init_matrix, RPY_matrix;
        Operate_tool::correct_coordinate(nh, init_coordinate, ref_coordinate, init_matrix);
        RPY_matrix.setRPY(RPY_correct[0], RPY_correct[1], RPY_correct[2]);
        matrix = RPY_matrix.inverse() * init_matrix;

        // Operate_tool::switch_controller("twist_controller","scaled_pos_joint_traj_controller", robot_name);

        // delay，Waiting for the above preparations to be complete
        ros::Duration(0.5).sleep();
        // msg.print();
        msg/=run_time;
        msg.print();
        tf::Vector3 world_linear_velocity(msg[0], msg[1], msg[2]);
        tf::Vector3 world_angular_velocity(msg[3], msg[4], msg[5]);

        // std::cout<<world_linear_velocity[0]<<" "<<world_linear_velocity[1]<<" "<<world_linear_velocity[2]<<std::endl;

        tf::Vector3 base_linear_velocity = world_linear_velocity * matrix;
        tf::Vector3 base_angular_velocity = world_angular_velocity * matrix;

        // std::cout<<base_linear_velocity[0]<<" "<<base_linear_velocity[1]<<" "<<base_linear_velocity[2]<<std::endl;

        realtime_pub->msg_.linear.x = base_linear_velocity.getX();
        realtime_pub->msg_.linear.y = base_linear_velocity.getY();
        realtime_pub->msg_.linear.z = base_linear_velocity.getZ();
        realtime_pub->msg_.angular.x = base_angular_velocity.getX();
        realtime_pub->msg_.angular.y = base_angular_velocity.getY();
        realtime_pub->msg_.angular.z = base_angular_velocity.getZ();
        realtime_pub->unlockAndPublish();
        realtime_pub->unlockAndPublish();
        realtime_pub->unlockAndPublish();
        printf("real x=%.6f y=%.6f z=%.6f\n",realtime_pub->msg_.linear.x,realtime_pub->msg_.linear.y,realtime_pub->msg_.linear.z);
        printf("angu x=%.6f y=%.6f z=%.6f\n",realtime_pub->msg_.angular.x,realtime_pub->msg_.angular.y,realtime_pub->msg_.angular.z);

        // std::cout << "--------wait---------" << std::endl;
        // double run_time = 0.2;
        // double run_time = msg.mod();
        // printf("start and run %.3lf\n",run_time);
        ros::Duration(run_time).sleep();
        // printf("stop\n");
        realtime_pub->msg_.linear.x = 0;
        realtime_pub->msg_.linear.y = 0;
        realtime_pub->msg_.linear.z = 0;
        realtime_pub->msg_.angular.x = 0;
        realtime_pub->msg_.angular.y = 0;
        realtime_pub->msg_.angular.z = 0;
        realtime_pub->unlockAndPublish();
        realtime_pub->unlockAndPublish();
        realtime_pub->unlockAndPublish();
        // Operate_tool::switch_controller("scaled_pos_joint_traj_controller","twist_controller", robot_name);
    }
    void twist_publish(ros::NodeHandle nh, ros::Publisher twist_pub, TwistMoveMsg msg, double run_time)
    {
        msg/=run_time;
        // msg.print();
        tf::Vector3 world_linear_velocity(msg[0], msg[1], msg[2]);
        tf::Vector3 world_angular_velocity(msg[3], msg[4], msg[5]);
        std_msgs::Float64MultiArray param;
        param.data.resize(6);
        for(int i=0;i<6;i++)param.data[i]=msg[i];
        twist_pub.publish(param);
        // printf("pub (%.6f,%.6f,%.6f)(%.6f,%.6f,%.6f)\n",param.data[0],param.data[1],param.data[2],param.data[3],param.data[4],param.data[5]);

        ros::Duration(run_time).sleep();
        // printf("stop\n");

        for(int i=0;i<6;i++)param.data[i]=0;
        twist_pub.publish(param);
    }
    void twist_dual_move(ros::NodeHandle nh, TwistMoveMsg msg,
                        double run_time, std::vector<double> RPY_correct, std::string ref_coordinate)
    {
        
        std::string left_twist_command_name, right_twist_command_name;
        left_twist_command_name = "/left_robot/twist_controller/command";
        right_twist_command_name = "/right_robot/twist_controller/command";

        std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> left_realtime_pub, right_realtime_pub;
        left_realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, left_twist_command_name, 4);
        right_realtime_pub = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, right_twist_command_name, 4);

        // Obtain the transformation matrix
        tf::Matrix3x3 left_matrix, right_matrix;
        tf::Matrix3x3 left_init_matrix, right_init_matrix, RPY_matrix;
        Operate_tool::correct_coordinate(nh, "/left_base", ref_coordinate, left_init_matrix);
        Operate_tool::correct_coordinate(nh, "/right_base", ref_coordinate, right_init_matrix);
        RPY_matrix.setRPY(RPY_correct[0], RPY_correct[1], RPY_correct[2]);
        left_matrix = RPY_matrix.inverse() * left_init_matrix;
        right_matrix = RPY_matrix.inverse() * right_init_matrix;

        // delay，Waiting for the above preparations to be complete
        msg/=run_time;
        ros::Duration(0.5).sleep();
        
        tf::Vector3 left_world_linear_velocity(msg[0], msg[1], msg[2]);
        tf::Vector3 left_world_angular_velocity(msg[3], msg[4], msg[5]);
        tf::Vector3 right_world_linear_velocity(msg[0], msg[1], msg[2]);
        tf::Vector3 right_world_angular_velocity(msg[3], msg[4], msg[5]);

        tf::Vector3 left_base_linear_velocity = left_world_linear_velocity * left_matrix;
        tf::Vector3 left_base_angular_velocity = left_world_angular_velocity * left_matrix;
        tf::Vector3 right_base_linear_velocity = right_world_linear_velocity * right_matrix;
        tf::Vector3 right_base_angular_velocity = right_world_angular_velocity * right_matrix;

        left_realtime_pub->msg_.linear.x = left_base_linear_velocity.getX();
        left_realtime_pub->msg_.linear.y = left_base_linear_velocity.getY();
        left_realtime_pub->msg_.linear.z = left_base_linear_velocity.getZ();
        left_realtime_pub->msg_.angular.x = left_base_angular_velocity.getX();
        left_realtime_pub->msg_.angular.y = left_base_angular_velocity.getY();
        left_realtime_pub->msg_.angular.z = left_base_angular_velocity.getZ();

        right_realtime_pub->msg_.linear.x = right_base_linear_velocity.getX();
        right_realtime_pub->msg_.linear.y = right_base_linear_velocity.getY();
        right_realtime_pub->msg_.linear.z = right_base_linear_velocity.getZ();
        right_realtime_pub->msg_.angular.x = right_base_angular_velocity.getX();
        right_realtime_pub->msg_.angular.y = right_base_angular_velocity.getY();
        right_realtime_pub->msg_.angular.z = right_base_angular_velocity.getZ();

        left_realtime_pub->unlockAndPublish();
        right_realtime_pub->unlockAndPublish();

        ros::Duration(run_time).sleep();
        left_realtime_pub->msg_.linear.x = 0;
        left_realtime_pub->msg_.linear.y = 0;
        left_realtime_pub->msg_.linear.z = 0;
        left_realtime_pub->msg_.angular.x = 0;
        left_realtime_pub->msg_.angular.y = 0;
        left_realtime_pub->msg_.angular.z = 0;
        right_realtime_pub->msg_.linear.x = 0;
        right_realtime_pub->msg_.linear.y = 0;
        right_realtime_pub->msg_.linear.z = 0;
        right_realtime_pub->msg_.angular.x = 0;
        right_realtime_pub->msg_.angular.y = 0;
        right_realtime_pub->msg_.angular.z = 0;
        left_realtime_pub->unlockAndPublish();
        right_realtime_pub->unlockAndPublish();
    }

}

namespace Modeling_tools
{
    Bowl::Bowl(geometry_msgs::Pose bottom_pose, double radius)
    {
        this->bottom_pose = bottom_pose;
        this->radius = radius;
    }

    geometry_msgs::Pose Bowl::Get_grindPose(double incline_angle, double rotation_angle)
    {
        // Obtain the bottom poseofinformation
        tf2::Quaternion orientation;
        tf2::fromMsg(bottom_pose.orientation, orientation);

        // Transform the posture of the bottom of the mortar toRPY
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        tf::Matrix3x3 ZYZ_Matrix;
        ZYZ_Matrix = Pose_Tool::matrix_ZYZ(rotation_angle, incline_angle, -1 * rotation_angle);
        double zyz_roll, zyz_pitch, zyz_yaw;
        ZYZ_Matrix.getRPY(zyz_roll, zyz_pitch, zyz_yaw);

        double temp_roll, temp_pitch, temp_yaw;
        temp_roll = roll + zyz_roll;
        temp_pitch = pitch + zyz_pitch;
        temp_yaw = yaw + zyz_yaw;

        // Translate into EnglishmodifyafterofEuler anglesconvert toquaternion
        tf2::Quaternion tf_orientation;
        tf_orientation.setRPY(temp_roll, temp_pitch, temp_yaw);

        geometry_msgs::Pose goal_pose;

        goal_pose.position.x = bottom_pose.position.x + radius * sin(incline_angle) * cos(rotation_angle);
        goal_pose.position.y = bottom_pose.position.y + radius * sin(incline_angle) * sin(rotation_angle);
        goal_pose.position.z = bottom_pose.position.z + radius * (1 - cos(incline_angle));
        goal_pose.orientation.x = tf_orientation.getX();
        goal_pose.orientation.y = tf_orientation.getY();
        goal_pose.orientation.z = tf_orientation.getZ();
        goal_pose.orientation.w = tf_orientation.getW();

        return goal_pose;
    }
}

namespace Kinematics_tools
{
    SingleArm_robot::SingleArm_robot(moveit::planning_interface::MoveGroupInterfacePtr mgptr)
    {
        move_group_ptr_ = mgptr;

        std::string robot_name;
        robot_name = mgptr->getName();

        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        joint_model_group_ = robot_model_->getJointModelGroup(robot_name);
    }

    // Settingsjointcornerdegree
    void SingleArm_robot::setJointPositions(const std::vector<double> &joint_positions)
    {
        robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
    }

    // Calculate forward kinematics
    Eigen::Affine3d SingleArm_robot::forwardKinematics()
    {
        return robot_state_->getGlobalLinkTransform(move_group_ptr_->getEndEffectorLink());
    }

    // Calculate Jacobian matrix
    Eigen::MatrixXd SingleArm_robot::computeJacobian()
    {
        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(joint_model_group_,
                                  robot_model_->getLinkModel(move_group_ptr_->getEndEffectorLink()),
                                  Eigen::Vector3d(0, 0, 0),
                                  jacobian);
        return jacobian;
    }
    // obtainCurrent joint angledegree
    std::vector<double> SingleArm_robot::getCurrentJointPositions()
    {
        return move_group_ptr_->getCurrentJointValues();
    }

    DualArm_robot::DualArm_robot(moveit::planning_interface::MoveGroupInterfacePtr mgptr)
    {
        move_group_ptr_ = mgptr;
        // Get joint group name
        std::string robotgroup_name = move_group_ptr_->getName();
        joint_model_group_ = move_group_ptr_->getCurrentState()->getJointModelGroup(robotgroup_name);
        moveit::core::RobotStatePtr current_state = move_group_ptr_->getCurrentState();

        // obtainPlanning teamofSub planning GroupName List
        const std::vector<std::string> &subgroup_names = joint_model_group_->getSubgroupNames();
        // Define twoaSub planning Group（Be sure to pay attention，whichayesleft，whichayesright）
        this->left_mgtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(subgroup_names[0]);
        this->right_mgtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(subgroup_names[1]);

        std::string robot_name, left_name, right_name;
        robot_name = move_group_ptr_->getName();
        left_name = left_mgtr->getName();
        right_name = right_mgtr->getName();

        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        joint_model_group_ = robot_model_->getJointModelGroup(robot_name);
        left_jmgp_ = robot_model_->getJointModelGroup(left_name);
        right_jmgp_ = robot_model_->getJointModelGroup(right_name);

        // obtainRW_transformandLW_transform
        std::string file_root =
            ros::package::getPath("manipulator") + "/data/yaml_data/";
        std::string filename = "RlW_transform.yaml";
        std::string use_filename = file_root + filename;
        YAML::Node node = YAML::LoadFile(use_filename);
        while (ros::ok()){
            if(node.IsNull()){
                ROS_ERROR_STREAM("Failed to load RlW_transform.yaml...");
                node = YAML::LoadFile(use_filename);
            }
            else{
                ROS_INFO_STREAM("Successed to load RlW_transform.yaml.");
                break;
            }
        }

        RW_transform.resize(6, 6);
        RW_transform.setZero();
        for (size_t i = 0; i < RW_transform.rows(); ++i) {
            for (size_t j = 0; j < RW_transform.cols(); ++j) {
                RW_transform(i, j) = node["RW_transform"][i][j].as<double>();
            }
        }
        LW_transform.resize(6, 6);
        LW_transform.setZero();
        for (size_t i = 0; i < LW_transform.rows(); ++i) {
            for (size_t j = 0; j < LW_transform.cols(); ++j) {
                LW_transform(i, j) = node["LW_transform"][i][j].as<double>();
            }
        }
        // monitorRW_transformandLW_transformyesNo, it has already been assigned a value
        if (RW_transform.isZero() || LW_transform.isZero())
        {
            ROS_ERROR_STREAM("RW_transform or LW_transform is NULL");
            ros::shutdown();
        }
        std::vector<double> current_joints = move_group_ptr_->getCurrentJointValues();
        this->setJointPositions(current_joints);
    }

    // Settingsjointcornerdegree
    void DualArm_robot::setJointPositions(const std::vector<double> &joint_positions)
    {
        robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
    }
    // Calculate forward kinematics
    Eigen::Affine3d DualArm_robot::left_forwardKinematics()
    {
        return robot_state_->getGlobalLinkTransform(left_mgtr->getEndEffectorLink());
    }
    Eigen::Affine3d DualArm_robot::right_forwardKinematics()
    {
        return robot_state_->getGlobalLinkTransform(right_mgtr->getEndEffectorLink());
    }
    // Calculate Jacobian matrix
    Eigen::MatrixXd DualArm_robot::left_computeJacobian()
    {
        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(left_jmgp_,
                                  robot_model_->getLinkModel(left_mgtr->getEndEffectorLink()),
                                  Eigen::Vector3d(0, 0, 0),
                                  jacobian);
        return LW_transform * jacobian;
    }
    // Calculate Jacobian matrix
    Eigen::MatrixXd DualArm_robot::right_computeJacobian()
    {
        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(right_jmgp_,
                                  robot_model_->getLinkModel(right_mgtr->getEndEffectorLink()),
                                  Eigen::Vector3d(0, 0, 0),
                                  jacobian);
        return RW_transform * jacobian;
    }
    // obtainCurrent joint angledegree
    std::vector<double> DualArm_robot::getCurrentJointPositions()
    {
        return move_group_ptr_->getCurrentJointValues();
    }

    void IKinverse_tool(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                        geometry_msgs::Pose target_pose, std::vector<double> &target_joints)
    {
        // loadUR5ofrobot model
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();

        const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(mgptr->getName());
        std::vector<double> current_joints = mgptr->getCurrentJointValues();
        kinematic_state->setJointGroupPositions(joint_model_group, current_joints);

        bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose);
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);
        }
        else
            ROS_WARN("No IK solution found");
    }
}

namespace Constraint_tools
{

    SingleArm_Constraint::SingleArm_Constraint(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                                               unsigned int cst_num) : ompl::base::Constraint(6, cst_num)
    {
        this->move_group_ptr = mgptr;
        this->constraint_num = cst_num;
        this->single_robot_ptr = std::make_shared<Kinematics_tools::SingleArm_robot>(mgptr);

        Init_joints = move_group_ptr->getCurrentJointValues();
        Init_pose = forwardKinematics(Eigen::Map<const Eigen::VectorXd>(Init_joints.data(), Init_joints.size()));
    }

    void SingleArm_Constraint::set_currentPJ(void)
    {
        Init_joints = move_group_ptr->getCurrentJointValues();
        Init_pose = forwardKinematics(Eigen::Map<const Eigen::VectorXd>(Init_joints.data(), Init_joints.size()));
    }

    void SingleArm_Constraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // hypothesis forwardKinematics Return to the end of the robotic armofposition
        geometry_msgs::Pose endpose = forwardKinematics(x);

        // calculateendtocenter of the sphereofdistance，andcheckityesNo equals ballofradius
        std::vector<double> return_values;
        constraint_formula(endpose, return_values);
        for (size_t i = 0; i < return_values.size(); i++)
        {
            out[i] = return_values[i];
        }
        // out[1] = xyzrpy_values[1] - Init_pose.position.y;
    }

    void SingleArm_Constraint::constraint_formula(geometry_msgs::Pose endpose, std::vector<double> &return_values) const
    {
        // -----lxyespositionx，aryesRPYcornerofr-----
        std::vector<double> xyzrpy_values;
        std::vector<double> rpy_values;
        rpy_values = Pose_Tool::self_getRPY(endpose.orientation);

        xyzrpy_values.push_back(endpose.position.x);
        xyzrpy_values.push_back(endpose.position.y);
        xyzrpy_values.push_back(endpose.position.z);
        xyzrpy_values.push_back(rpy_values[0]);
        xyzrpy_values.push_back(rpy_values[1]);
        xyzrpy_values.push_back(rpy_values[2]);

        return_values.push_back(xyzrpy_values[0] - Init_pose.position.x);
    }

    void SingleArm_Constraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        // 0.hypothesis forwardKinematics Return to the end of the robotic armofposition
        geometry_msgs::Pose endpose = forwardKinematics(x);
        // 1.Translate into Englishrobotic armofjointcornerassignmenttojoint_positions
        std::vector<double> joint_values(x.data(), x.data() + x.size());
        single_robot_ptr->setJointPositions(joint_values);

        // 2.Calculate the joint angleofJacobian matrix
        Eigen::MatrixXd jacobian_matrix = single_robot_ptr->computeJacobian();

        // 3.According to constraintsobtainhandleafterofJacobi
        // （Only constraintxaxis,Then onlyextractJacobimatrixofitem1line）
        Eigen::MatrixXd return_matrix(constraint_num, jacobian_matrix.cols());
        jacobian_formula(jacobian_matrix, endpose, return_matrix);

        // 4. output
        out = return_matrix;
    }
    void SingleArm_Constraint::jacobian_formula(Eigen::MatrixXd jacobian_matrix, geometry_msgs::Pose endpose, 
                                                Eigen::MatrixXd &return_matrix) const
    {
        return_matrix.row(0) = jacobian_matrix.row(0);
    }

    geometry_msgs::Pose SingleArm_Constraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        std::vector<double> joint_positions(x.data(), x.data() + x.size());
        single_robot_ptr->setJointPositions(joint_positions);

        Eigen::Affine3d end_effector_pose = single_robot_ptr->forwardKinematics();
        Eigen::Quaterniond quaternion(end_effector_pose.rotation());

        geometry_msgs::Pose robot_pose;

        robot_pose.position.x = end_effector_pose.translation()(0, 0);
        robot_pose.position.y = end_effector_pose.translation()(1, 0);
        robot_pose.position.z = end_effector_pose.translation()(2, 0);
        robot_pose.orientation.x = quaternion.x();
        robot_pose.orientation.y = quaternion.y();
        robot_pose.orientation.z = quaternion.z();
        robot_pose.orientation.w = quaternion.w();

        return robot_pose;
    }

    // add：checkstatusyesIs it effective（Avoid obstacles）
    bool DualArm_Constraint::isValid(const ompl::base::State* state) const
    {
        Eigen::VectorXd q(12);
        for(int i=0; i<12; i++){
            q[i] = state->as<ompl::base::ConstrainedStateSpace::StateType>()->data()[i];
        }

        moveit::core::RobotState rs = planning_scene_ptr->getCurrentState();
        const moveit::core::JointModelGroup* jmg = rs.getJointModelGroup("dual_robot");

        double pos;
        for(int i=0; i<12; i++){
            pos = q[i];
            rs.setJointPositions(jmg->getActiveJointModels()[i], &pos);
        }

        rs.update();

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;

        planning_scene_ptr->checkCollisionUnpadded(collision_request, collision_result, rs);

        return !collision_result.collision;
    }
    DualArm_Constraint::DualArm_Constraint(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                                           unsigned int cst_num) : ompl::base::Constraint(12, cst_num)
    {
        auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        bool success = psm->requestPlanningSceneState("/get_planning_scene");
        ROS_INFO_STREAM("Request planning scene " << (success ? "succeed." : "failed."));
        planning_scene_monitor::LockedPlanningSceneRW ls(psm);
        planning_scene_ptr = ls->diff();
        planning_scene_ptr->decoupleParent();
        planning_scene_ptr->setName(ls->getName());

        this->move_group_ptr = mgptr;
        this->constraint_num = cst_num;

        this->dual_robot_ptr = std::make_shared<Kinematics_tools::DualArm_robot>(mgptr);

        Init_joints = move_group_ptr->getCurrentJointValues();

        Init_pose_left = forwardKinematics(Eigen::Map<const Eigen::VectorXd>(Init_joints.data(), Init_joints.size()), false);
        Init_pose_right = forwardKinematics(Eigen::Map<const Eigen::VectorXd>(Init_joints.data(), Init_joints.size()), true);
    }

    bool DualArm_Constraint::project(Eigen::Ref<Eigen::VectorXd> x) const
    {
        // Newton's method
        unsigned int iter = 0;
        double norm = 0;
        Eigen::VectorXd f(this->getCoDimension());
        Eigen::MatrixXd j(this->getCoDimension(), n_);

        const double squaredTolerance = this->tolerance_ * this->tolerance_;
        function(x, f);
        std::cout << "-" << "hhhh:" << f.squaredNorm() << std::endl;
        std::cout << "-" << "maxIterations_:" << maxIterations_ << std::endl;
        std::cout << "-" << "squaredTolerance:" << squaredTolerance << std::endl;
        // Operate_tool::interrupt_judge();
        while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
        {
            this->jacobian(x, j);
            x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            this->function(x, f);
            std::cout << "what:" << iter << std::endl;
            std::cout << "--------" << f.squaredNorm() << "---------" << std::endl;
        }
        std::cout << "end!" << std::endl;

        return norm < squaredTolerance;
    }
    
    void DualArm_Constraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // hypothesis forwardKinematics Return to the end of the robotic armofposition
        geometry_msgs::Pose endpose_left = forwardKinematics(x, false);
        geometry_msgs::Pose endpose_right = forwardKinematics(x, true);

        // calculateendtocenter of the sphereofdistance，andcheckityesNo equals ballofradius
        std::vector<double> return_values;
        constraint_formula(endpose_left, endpose_right, return_values);
        for (size_t i = 0; i < return_values.size(); i++)
        {
            out[i] = return_values[i];
            // std::cout << out[i] << " ";
        }
        // std::cout << std::endl;
    }

    void DualArm_Constraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        // 0.hypothesis forwardKinematics Return to the end of the robotic armofposition
        geometry_msgs::Pose endpose_left = forwardKinematics(x, false);
        geometry_msgs::Pose endpose_right = forwardKinematics(x, true);

        // 1.Translate into Englishrobotic armofjointcornerassignmenttojoint_positions
        std::vector<double> joint_values(x.data(), x.data() + x.size());
        dual_robot_ptr->setJointPositions(joint_values);

        // 2.Calculate the joint angleofJacobian matrix
        Eigen::MatrixXd left_jacobian_matrix = dual_robot_ptr->left_computeJacobian();
        Eigen::MatrixXd right_jacobian_matrix = dual_robot_ptr->right_computeJacobian();

        // 3.According to constraintsobtainhandleafterofJacobi
        // （Only constraintxaxis,Then onlyextractJacobimatrixofitem1line）
        Eigen::MatrixXd return_matrix(constraint_num, (left_jacobian_matrix.cols() + right_jacobian_matrix.cols()));
        jacobian_formula(left_jacobian_matrix, right_jacobian_matrix, endpose_left, endpose_right, return_matrix);

        // 4. output
        out = return_matrix;
    }

    geometry_msgs::Pose DualArm_Constraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &x, bool LR_flag) const
    {
        std::vector<double> joint_positions(x.data(), x.data() + x.size());

        dual_robot_ptr->setJointPositions(joint_positions);

        Eigen::Affine3d end_effector_pose;
        if (!LR_flag)
            end_effector_pose = dual_robot_ptr->left_forwardKinematics();
        else
            end_effector_pose = dual_robot_ptr->right_forwardKinematics();

        Eigen::Quaterniond quaternion(end_effector_pose.rotation());

        geometry_msgs::Pose robot_pose;

        robot_pose.position.x = end_effector_pose.translation()(0, 0);
        robot_pose.position.y = end_effector_pose.translation()(1, 0);
        robot_pose.position.z = end_effector_pose.translation()(2, 0);
        robot_pose.orientation.x = quaternion.x();
        robot_pose.orientation.y = quaternion.y();
        robot_pose.orientation.z = quaternion.z();
        robot_pose.orientation.w = quaternion.w();

        return robot_pose;
    }
    void DualArm_Constraint::constraint_formula(geometry_msgs::Pose endpose_left, 
                                                geometry_msgs::Pose endpose_right, 
                                                std::vector<double> &return_values) const
    {
        ///////////////////////////////////////////////////////////////
        /*--------------1. Calculate positional constraints-------------------*/
        // 1. obtainoRE1, And the initialinit_oRE1
        Eigen::MatrixXd left_oRE1_matrix(3, 3), init_oRE1_matrix(3, 3);
        tf::Quaternion left_quaternion, init_quaternion;
        tf::quaternionMsgToTF(endpose_left.orientation, left_quaternion);
        tf::quaternionMsgToTF(Init_pose_left.orientation, init_quaternion);
        tf::Matrix3x3 left_R_matrix(left_quaternion); // createonea3x3ofrotation matrix
        tf::Matrix3x3 init_R_matrix(init_quaternion); // createonea3x3ofrotation matrix
        for(size_t i = 0; i < 3; i++)
            for(size_t j = 0; j < 3; j++)
            {
                left_oRE1_matrix(i, j) = left_R_matrix[i][j];
                init_oRE1_matrix(i, j) = init_R_matrix[i][j];
            }
        // 2. relative position
        Eigen::Vector3d realtive_LtoR, init_LtoR;  //3*1 ?
        realtive_LtoR << (endpose_right.position.x - endpose_left.position.x), 
                         (endpose_right.position.y - endpose_left.position.y), 
                         (endpose_right.position.z - endpose_left.position.z);
        init_LtoR << (Init_pose_right.position.x - Init_pose_left.position.x), 
                     (Init_pose_right.position.y - Init_pose_left.position.y), 
                     (Init_pose_right.position.z - Init_pose_left.position.z);
        // 3. Position constraint result
        Eigen::Vector3d result_fpq;   
        result_fpq = (left_oRE1_matrix.transpose() * realtive_LtoR - init_oRE1_matrix.transpose() * init_LtoR);
        // result_fpq = realtive_LtoR - init_LtoR;
        /*----------------------------------------------*/
        /*--------------2. Calculate attitude constraints-------------------*/
        Eigen::Vector4d left_wxyz_E1, right_wxyz_E2;
        Eigen::Vector4d left_wxyz_E1_inverse;
        Eigen::Vector4d wxyz_init_LtoR;
        Eigen::Vector4d left_init_wxyz_E1_inverse, right_init_wxyz_E2;
        left_wxyz_E1 << endpose_left.orientation.w,
                        endpose_left.orientation.x,
                        endpose_left.orientation.y,
                        endpose_left.orientation.z;
        left_wxyz_E1_inverse << endpose_left.orientation.w,
                                -1 * endpose_left.orientation.x,
                                -1 * endpose_left.orientation.y,
                                -1 * endpose_left.orientation.z;
        right_wxyz_E2 << endpose_right.orientation.w,
                         endpose_right.orientation.x,
                         endpose_right.orientation.y,
                         endpose_right.orientation.z;  
        left_init_wxyz_E1_inverse << Init_pose_left.orientation.w,
                                     -1 * Init_pose_left.orientation.x,
                                     -1 * Init_pose_left.orientation.y,
                                     -1 * Init_pose_left.orientation.z;         
        right_init_wxyz_E2 << Init_pose_right.orientation.w,
                              Init_pose_right.orientation.x,
                              Init_pose_right.orientation.y,
                              Init_pose_right.orientation.z;  
        // $t^-1
        wxyz_init_LtoR = Constraint_tools::Constraint_M(left_init_wxyz_E1_inverse, true) * right_init_wxyz_E2;  
        // wxyz_init_LtoR = Constraint_tools::Constraint_M(right_init_wxyz_E2_inverse, false) * left_init_wxyz_E1;  
        for(size_t i = 1; i < 4; i++) 
            wxyz_init_LtoR(i) = -1 * wxyz_init_LtoR(i);
        Eigen::Vector4d result_temp_foq; // （1, 0, 0, 0）T
        result_temp_foq =  Constraint_tools::Constraint_M(wxyz_init_LtoR, true) * 
                           Constraint_tools::Constraint_M(right_wxyz_E2, false) * left_wxyz_E1_inverse;
        // result_temp_foq =  Constraint_tools::Constraint_M(wxyz_init_LtoR, true) * 
        //                    Constraint_tools::Constraint_M(right_wxyz_E2_inverse, false) * left_wxyz_E1;
        Eigen::Vector3d result_foq;  
        // Translate into Englishresult_temp_foqAssign the last three lines toresult_foq
        result_foq = result_temp_foq.segment<3>(1);
        /*----------------------------------------------*/
        for(size_t i = 0; i < 6; i++)
        {
            if(i < 3)
                // return_values.push_back(0.0);
                return_values.push_back(result_fpq(i));
            else
                return_values.push_back(result_foq(i -3));
                // return_values.push_back(0.0);
        }       
        ///////////////////////////////////////////////////////////////
    }

    void DualArm_Constraint::jacobian_formula(Eigen::MatrixXd left_jacobian_matrix,
                                              Eigen::MatrixXd right_jacobian_matrix,
                                              geometry_msgs::Pose endpose_left,
                                              geometry_msgs::Pose endpose_right,
                                              Eigen::MatrixXd &return_matrix) const
    {
        ///////////////////////////////////////////////////////////////
        /*--------------1. Calculate the Jacobian matrix of the position-------------------*/
        Eigen::MatrixXd left_pos_temp_jacobian(3, 6), right_pos_temp_jacobian(3, 6);
        Eigen::MatrixXd position_temp_jacobian(3, 12), position_jacobian(3, 12);
        // 1. obtainoRE1
        Eigen::MatrixXd left_oRE1_matrix(3, 3);
        tf::Quaternion left_quaternion;
        tf::quaternionMsgToTF(endpose_left.orientation, left_quaternion);
        tf::Matrix3x3 left_R_matrix(left_quaternion); // createonea3x3ofrotation matrix
        for(size_t i = 0; i < 3; i++)
            for(size_t j = 0; j < 3; j++)
                left_oRE1_matrix(i, j) = left_R_matrix[i][j];

        // 2. relative position
        Eigen::Vector3d realtive_LtoR;  //3*1 withleftThe end of the arm serves as a reference，rightrelative toleft
        realtive_LtoR << (endpose_right.position.x - endpose_left.position.x), 
                         (endpose_right.position.y - endpose_left.position.y), 
                         (endpose_right.position.z - endpose_left.position.z);

        // 3. Temporary Jacobian Matrix                
        Eigen::MatrixXd left_JvE1(3, 6), left_JwE1(3, 6);
        Eigen::MatrixXd right_JvE2(3, 6), right_JwE2(3, 6);
        left_JvE1 = left_jacobian_matrix.block(0, 0, 3, 6);
        left_JwE1 = left_jacobian_matrix.block(3, 0, 3, 6);
        right_JvE2 = right_jacobian_matrix.block(0, 0, 3, 6);
        right_JwE2 = right_jacobian_matrix.block(3, 0, 3, 6);
        // S(&p)JwE1 - JvE1
        left_pos_temp_jacobian = Constraint_tools::Constraint_S(realtive_LtoR) * left_JwE1 - left_JvE1;
        // JvE2
        right_pos_temp_jacobian = right_JvE2;
        // [S(&p)JwE1 - JvE1, JvE2]
        position_temp_jacobian.block(0, 0, 3, 6) = left_pos_temp_jacobian;
        position_temp_jacobian.block(0, 6, 3, 6) = right_pos_temp_jacobian;
        // O RT E1[S(&p)JwE1 - JvE1, JvE2]
        position_jacobian = left_oRE1_matrix.transpose() * position_temp_jacobian;
        // position_jacobian = position_temp_jacobian;
        /*---------------------------------------------------*/

        /*--------------2. Calculate the Jacobian matrix of attitude-------------------*/
        Eigen::Vector4d left_wxyz_E1, right_wxyz_E2;
        Eigen::Vector4d left_wxyz_E1_inverse;
        Eigen::Vector4d wxyz_init_LtoR;
        Eigen::Vector4d left_init_wxyz_E1_inverse, right_init_wxyz_E2;
        left_wxyz_E1 << endpose_left.orientation.w,
                        endpose_left.orientation.x,
                        endpose_left.orientation.y,
                        endpose_left.orientation.z;
        left_wxyz_E1_inverse << endpose_left.orientation.w,
                                -1 * endpose_left.orientation.x,
                                -1 * endpose_left.orientation.y,
                                -1 * endpose_left.orientation.z;
        right_wxyz_E2 << endpose_right.orientation.w,
                         endpose_right.orientation.x,
                         endpose_right.orientation.y,
                         endpose_right.orientation.z;          
        left_init_wxyz_E1_inverse << Init_pose_left.orientation.w,
                                     -1 * Init_pose_left.orientation.x,
                                     -1 * Init_pose_left.orientation.y,
                                     -1 * Init_pose_left.orientation.z;
        right_init_wxyz_E2 << Init_pose_right.orientation.w,
                              Init_pose_right.orientation.x,
                              Init_pose_right.orientation.y,
                              Init_pose_right.orientation.z;        

        Eigen::MatrixXd left_ori_temp_jacobian(4, 6), right_ori_temp_jacobian(4, 6);
        Eigen::MatrixXd orientation_temp_jacobian(4, 12), orientation_final_jacobian(4, 12);
        Eigen::MatrixXd orientation_jacobian(3, 12);
        Eigen::MatrixXd left_temp_q(4, 6), right_temp_q(4, 6);
        // (HT($E1)JwE1)^-1
        left_temp_q = Constraint_tools::Constraint_H(left_wxyz_E1).transpose() * left_JwE1;
        left_temp_q.block(1, 0, 3, 6) = -1 * left_temp_q.block(1, 0, 3, 6);
        // HT($E2)JwE2
        right_temp_q = Constraint_tools::Constraint_H(right_wxyz_E2).transpose() * right_JwE2;

        // $t^-1
        wxyz_init_LtoR = Constraint_tools::Constraint_M(left_init_wxyz_E1_inverse, true) * right_init_wxyz_E2;
        for(size_t i = 1; i < 4; i++) 
            wxyz_init_LtoR(i) = -1 * wxyz_init_LtoR(i);
        // MR($E2)(HT($E1)JwE1)^-1
        left_ori_temp_jacobian = Constraint_tools::Constraint_M(right_wxyz_E2, false) * left_temp_q;
        // Ml($E1^-1)HT($E2)JwE2
        right_ori_temp_jacobian = Constraint_tools::Constraint_M(left_wxyz_E1_inverse, true) * right_temp_q;
        // [MR($E2)(HT($E1)JwE1)^-1, Ml($E1^-1)HT($E2)JwE2]
        orientation_temp_jacobian.block(0, 0, 4, 6) = left_ori_temp_jacobian;
        orientation_temp_jacobian.block(0, 6, 4, 6) =  right_ori_temp_jacobian;
        // 1/2 Ml($t^-1)[MR($E2)(HT($E1)JwE1)^-1, Ml($E1^-1)HT($E2)JwE2]
        orientation_final_jacobian = 0.5 * Constraint_tools::Constraint_M(wxyz_init_LtoR, true) * orientation_temp_jacobian;

        orientation_jacobian = orientation_final_jacobian.block(1, 0, 3, 12);
        /*---------------------------------------------------*/
        for(size_t i = 0; i < 6; i++)
        {
            if(i < 3)
                return_matrix.row(i) = position_jacobian.row(i);
            else
                return_matrix.row(i) = orientation_jacobian.row(i - 3);
        }
        ///////////////////////////////////////////////////////////////
    }

    Eigen::MatrixXd Constraint_S(Eigen::Vector3d xyz)  // 3*3
    {
        Eigen::MatrixXd return_S(3,3);
        return_S << 0, -1 * xyz(2), xyz(1),
                    xyz(2), 0, -1 * xyz(0),
                    -1 * xyz(1), xyz(0), 0;

        return return_S;
    }

    Eigen::MatrixXd Constraint_M(Eigen::Vector4d wxyz, bool LR_flag)  // 4*4
    {
        Eigen::MatrixXd return_M(4,4);
        if(LR_flag)
        {   // left
            return_M << wxyz(0), -1 * wxyz(1), -1 * wxyz(2), -1 * wxyz(3),
                        wxyz(1), wxyz(0), -1 * wxyz(3), wxyz(2),
                        wxyz(2), wxyz(3), wxyz(0), -1 * wxyz(1),
                        wxyz(3), -1 * wxyz(2), wxyz(1), wxyz(0);
        }
        else
        {   // right
            return_M << wxyz(0), -1 * wxyz(1), -1 * wxyz(2), -1 * wxyz(3),
                        wxyz(1), wxyz(0), wxyz(3), -1 * wxyz(2),
                        wxyz(2), -1 * wxyz(3), wxyz(0), wxyz(1),
                        wxyz(3), wxyz(2), -1 * wxyz(1), wxyz(0);            
        }
        return return_M;
    }
    Eigen::MatrixXd Constraint_H(Eigen::Vector4d wxyz)  // 3*4
    {
        Eigen::MatrixXd return_H(3,4);
        return_H << -1 * wxyz(1), wxyz(0), -1 * wxyz(3), wxyz(2),
                    -1 * wxyz(2), wxyz(3), wxyz(0), -1 * wxyz(1),
                    -1 * wxyz(3), -1 * wxyz(2), wxyz(1), wxyz(0);

        return return_H;
    }

    void CloseChain_cst_move(moveit::planning_interface::MoveGroupInterfacePtr mgptr,
                             std::vector<double> target_joints, double tolerance_value,
                             unsigned int cst_num,
                             moveit_msgs::RobotTrajectory& trajectory,
                             bool execute_flag)
    {
        //--------------------------------------------------------------
        // 0.createconstraint（The constraint is aboveofClassclass）
        auto constraint = std::make_shared<Constraint_tools::DualArm_Constraint>(mgptr, cst_num);
        //--------------------------------------------------------------

        // Obtain the current joint values of the robotic arm----------
        std::vector<double> current_joints = mgptr->getCurrentJointValues();

        // 1.createOMPLstate space
        auto rvss = std::make_shared<ompl::base::RealVectorStateSpace>(12);
        rvss->setBounds(-2 * M_PI, 2 * M_PI); // Settingsjointcornerdegreescope
        // 2.Settingstoleratedegree
        constraint->setTolerance(tolerance_value);
        constraint->setMaxIterations(500);
        // 3.Composite constraint state space
        ////////////////////////////////////////////////
        // auto css = std::make_shared<ompl::base::ProjectedStateSpace>(rvss, constraint);
        auto css = std::make_shared<ompl::base::AtlasStateSpace>(rvss, constraint);
        // alpha: The chart is validregionInternal chartandmanifoldbetweenofMaximum allowable angledegree。Must be in (0, pi/2) within the scope of。default pi/16。
        css->setAlpha(M_PI/16);   
        // epsilon: The chart is validregionChineseofpointanditsinmanifoldupofBetween projectionsofmaximum allowabledistance。default value 0.1。
        css->setEpsilon(0.05);
        // Rho: Settings rho，namelyThe chart is validofMaximum radius。default value 0.1。
        css->setRho(1.0);
        // Exploration: Settingsexploreparameter，Used for adjusting and refining（Sampling within a known area）And exploration（Sampling on the boundary）betweenofbalance。
        //              The effective value is [0,1) within the scope of，itsChinese 0 Indicating complete refinement，1 Indicate all explorations。default value 0.5
        css->setExploration(0.75);
        // MaxChartsPerExtension: limitmanufactureonetimetraverseChinesecancreateofNumber of charts。default 200。
        css->setMaxChartsPerExtension(200);
        ////////////////////////////////////////////////
        auto csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(css);
        auto ss = std::make_shared<ompl::geometric::SimpleSetup>(csi);
        // 4.SettingsstartJoint value
        Eigen::VectorXd sv(12), gv(12);
        sv = Eigen::VectorXd::Map(current_joints.data(), current_joints.size());
        gv = Eigen::VectorXd::Map(target_joints.data(), target_joints.size());

        ompl::base::ScopedState<> start(css);
        ompl::base::ScopedState<> goal(css);
        start->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(sv);
        goal->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(gv);
        ss->setStartAndGoalStates(start, goal);
        ////////////////////////////////////////////////
        // auto pp = std::make_shared<ompl::geometric::RRT>(csi);
        css->anchorChart(start.get());
        auto pp = std::make_shared<ompl::geometric::RRTConnect>(csi);
        ////////////////////////////////////////////////
        ss->setPlanner(pp);
        ss->setup();

        ss->setStateValidityChecker([&constraint](const ompl::base::State *state) {
            return constraint->isValid(state);
        });
        // 5.Define maximum planning time
        ompl::base::PlannerStatus stat = ss->solve(50.);
        if (stat)
        {
            // 1.Path simplification、obtainpath、interpolation
            ss->simplifySolution(5.);
            auto path = ss->getSolutionPath();
            path.interpolate();
            // 2.Print Path
            std::cout << "Found solution:" << std::endl;
            path.print(std::cout);

            //////////////////////////--------Customize optimization trajectory--------------////////////////////////////////////////
            // 3.Translate into EnglishOMPLPath conversion toMoveIt!path
            std::vector<robot_state::RobotStatePtr> moveit_path 
                                = Operate_tool::omplPathToMoveItPath(path, mgptr);
            // 4.1 Obtain smoothnesstrajectory
            Operate_tool::Smooth_trajectory(trajectory, 0.01, moveit_path, mgptr);
            // 4.2 deleteitemoneatrajectorypoint（becauseforitsandsecondaonesample）
            trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
            // 4.3 deletetimestamponesampleoftrajectorypoint
            Operate_tool::removeDuplicateTimeStamps(trajectory);
            ////////////////////////////////////////////////////////////////////////////////////////////////
            Operate_tool::interrupt_judge(execute_flag);

            // 8.Execution path
            if (execute_flag)
            {
                ROS_INFO_NAMED("CloseChain_move", "Planning and execution of the movement succeeded!");
                mgptr->execute(trajectory);
            }
            else
            {
                ROS_INFO_NAMED("CloseChain_move", "Only planning of the movement realizes!");
            }            
            
        }
        else
            OMPL_WARN("No solution found!");
    }
}
