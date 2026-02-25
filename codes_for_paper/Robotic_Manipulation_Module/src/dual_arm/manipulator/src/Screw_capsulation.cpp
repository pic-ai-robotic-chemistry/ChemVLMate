#include "manipulator/robot_arm.h"

bool debug_enable_flag = false;
bool record_traj_flag = false;
bool Traj_Record_flag = true;

namespace Robot_capsulation
{
    double Robot_operation::Screw_tare_force_z(void){
        std_msgs::Float64 tare_force_z;
        tare_force_z.data = ft_transition_ptr->left_force_z_filter.val()+bottle_weight;
        // printf("pub tare force z %.3f\n",tare_force_z.data);
        tare_force_z_pub.publish(tare_force_z);
        return tare_force_z.data;
    }
    std::string Robot_operation::Screw_loosen(void){
        std::string result;
        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        left_mgtr->setPlanningPipelineId("ompl");
        left_mgtr->setPlannerId("RRTConnect");
        ft_transition_ptr->left_force_z_filter.clear();
        if(!Operate_tool::switch_controller("selfdefined_trajectory_controller","scaled_pos_joint_traj_controller", "left_robot"))
            return "error";
        ros::Duration(0.5).sleep();

        bottle_weight=std::max(0.0,-ft_transition_ptr->left_force_z_filter.val());//ZAxis positive upward
        ROS_INFO("Whole bottle weight %.3f N\n",bottle_weight);
        Analysis_Command("A R PTP J 0.05 0.05 screw_loosen",false);
        geometry_msgs::WrenchStamped left_wrench_msg;
        try{
            // Target torque value
            left_wrench_msg.header.stamp = ros::Time::now();
            left_wrench_msg.header.frame_id = "left_tool0";//restrictionexerciseportion，Only upward
            
            // left_wrench_msg.wrench.force.y = 3*sin(50.0/180.0*acos(-1)); // Setting Poweryportion,negativevaluePull down
            // left_wrench_msg.wrench.force.z = 3*cos(50.0/180.0*acos(-1));
            left_wrench_pub.publish(left_wrench_msg);
            left_wrench_pub.publish(left_wrench_msg);
            left_wrench_pub.publish(left_wrench_msg);
            ros::Duration(0.5).sleep();
            // //Bottle cap touch detection
            ros::Time start_time = ros::Time::now();
            // do{
            //     ros::Duration(0.05).sleep();
            //     ROS_INFO("Probe the bottle cap into the center peelzaxial force= %.3f",Screw_tare_force_z());
            //     if ((ros::Time::now() - start_time).toSec() > 5.0)
            //         throw std::runtime_error("Bottle cap touch detection timeout！");
            // }while(Screw_tare_force_z()>-5);
            ROS_INFO("!!!Successfully touched the bottle cap peelzaxial force= %.3f",Screw_tare_force_z());
    
            if (R_RGIC_flag){//Clamp the bottle cap tightly
                result=gripper_ctrl(right_gripper_client,100,10,0,-1,-1,99999999,0);
                gripper_ctrl(left_gripper_client,-1,-1,-1,-1,-1,99999999,0);
                if(result!="clamping"){
                    ROS_ERROR("!!!Clamp the bottle cap tightlyfailure，Terminate operation status:%s",result.c_str());
                    gripper_ctrl(right_gripper_client,-1,-1,1000,-1,-1,99999999,0);
                    throw std::runtime_error("Clamp the bottle cap tightlyfailure，Terminate operation");
                }
                ros::Duration(0.5).sleep();
            }
            else{
                ROS_ERROR("RGIC is not open!!!");
                result="offline";
                throw std::runtime_error("RGICNot connected");
            }
    
            //Loose bottle cap，most1080degree
            gripper_ctrl_asyn(right_gripper_client,-1,70,-1,100,10,99999999,1500);
            gripper_ctrl(left_gripper_client,-1,-1,-1,-1,-1,99999999,0);
            ros::Duration(0.05).sleep();
    
            // left_mgtr->setPlanningPipelineId("ompl");
            // left_mgtr->setPlannerId("RRTConnect");
            // ft_transition_ptr->left_force_z_filter.clear();
            // if(!Operate_tool::switch_controller("selfdefined_trajectory_controller","scaled_pos_joint_traj_controller", "left_robot"))
            //     return "error";
            // ros::Duration(0.5).sleep();

            // Target torque value
            left_wrench_msg.header.stamp = ros::Time::now();
            left_wrench_msg.header.frame_id = "left_tool0";
            left_wrench_msg.wrench.force.y = -3*sin(50.0/180.0*acos(-1)); // Setting Poweryportion,negativevaluePull down
            left_wrench_msg.wrench.force.z = -3*cos(50.0/180.0*acos(-1));
            left_wrench_pub.publish(left_wrench_msg);
            left_wrench_pub.publish(left_wrench_msg);
            left_wrench_pub.publish(left_wrench_msg);
    
            ros::Duration(0.5).sleep();
            int loosen_count=0;
            start_time = ros::Time::now();
            do{
                ros::Duration(0.1).sleep();
                ROS_INFO("Loose bottle capChinese(%.1lf/15s) peelzaxial force= %.3f",(double)(ros::Time::now()-start_time).toSec(), Screw_tare_force_z());
                if(Screw_tare_force_z()<2)loosen_count++;
                if ((ros::Time::now() - start_time).toSec() > 15.0){
                    break;
                    // result="error-timeout";
                    // throw std::runtime_error("Loose bottle cap timeout！");
                }
            }while(loosen_count<10);
            ROS_INFO("!!!stopLoose bottle cap peelzaxial force= %.3f",Screw_tare_force_z());
    
            //Forcefully stop rotation
            ros::Duration(1.5).sleep();
            if(gripper_stop(right_gripper_client)!="clamping"){
                result="error-lost";
                throw std::runtime_error("Bottle cap falls off!");
            }
            gripper_ctrl(left_gripper_client,-1,-1,-1,-1,-1,99999999,0);
        }catch(const std::runtime_error& e){
            ROS_ERROR("alarm %s\n",e.what());
            gripper_ctrl_asyn(right_gripper_client,-1,-1,850,-1,-1,99999999,0);
        }
        ros::Duration(0.5).sleep();
        bottle_weight=-50;//Clear bottle data
        Screw_tare_force_z();
        left_wrench_msg.header.stamp = ros::Time::now();
        left_wrench_msg.header.frame_id = "left_tool0";
        left_wrench_msg.wrench.force.y = 0.0; // Resilience'syportion
        left_wrench_msg.wrench.force.z = 0.0; // Resilience'syportion
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);

        gripper_ctrl_asyn(right_gripper_client,-1,-1,-1,50,100,99999999,0);

        gripper_ctrl(left_gripper_client,-1,-1,-1,-1,-1,99999999,0);
        if(!Operate_tool::switch_controller("scaled_pos_joint_traj_controller","selfdefined_trajectory_controller", "left_robot")){
            while(1){
                ROS_ERROR("Unable to switch controllers，The robotic arm may lose control");
                ros::Duration(0.1).sleep();
            }
        }


        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
        left_mgtr->setPlannerId("PTP");

        ros::Duration(0.5).sleep();
        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        return result=="clamping"?"success":result;
    }
    std::string Robot_operation::Screw_tighten(){
        std::string result;
        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        Analysis_Command("A R LIN J 0.05 0.05 screw_tighten",false);
        left_mgtr->setPlanningPipelineId("ompl");
        left_mgtr->setPlannerId("RRTConnect");
        bottle_weight=-ft_transition_ptr->left_force_z_filter.val();//ZAxis positive upward
        ROS_INFO("Bottle weight（No bottle cap） %.3f N\n",bottle_weight);
        if(!Operate_tool::switch_controller("selfdefined_trajectory_controller","scaled_pos_joint_traj_controller", "left_robot"))
            return "error";
        
        ros::Duration(0.5).sleep();

        if (R_RGIC_flag){//twist the bottle cap，most3600degree，Blocked rotation stops
            result=gripper_ctrl_asyn(right_gripper_client,100,-1,-1,20,10,-1800,0);
        }
        else{
            ROS_ERROR("RGIC is not open!!!");
            result="offline";
        }

        // Target torque value
        geometry_msgs::WrenchStamped left_wrench_msg;
        left_wrench_msg.header.stamp = ros::Time::now();
        left_wrench_msg.header.frame_id = "left_tool0";
        left_wrench_msg.wrench.force.y = 3*sin(50.0/180.0*acos(-1)); // Setting Poweryportion,positivevalueTop up
        left_wrench_msg.wrench.force.z = 3*cos(50.0/180.0*acos(-1));
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);

        if (R_RGIC_flag){//twist the bottle cap，most3600degree，Blocked rotation stops
            result=gripper_ctrl(right_gripper_client,-1,-1,-1,-1,-1,-1800,0);
            if(result!="clamping&blocking" && result!="clamping"){
                ROS_ERROR("!!!Tightening bottle cap failed，Terminate operation status:%s",result.c_str());
                gripper_ctrl(right_gripper_client,-1,-1,1000,-1,-1,99999999,0);
                result="error-timeout";
            }
            ros::Duration(0.05).sleep();
        }
        else{
            ROS_ERROR("RGIC is not open!!!");
            result="offline";
        }
        ROS_INFO("The bottle cap has been tightened\n");
        left_wrench_msg.header.stamp = ros::Time::now();
        left_wrench_msg.header.frame_id = "left_tool0";
        left_wrench_msg.wrench.force.y = 0.0; // Resilience'syportion
        left_wrench_msg.wrench.force.z = 0.0; // Resilience'szportion
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);
        left_wrench_pub.publish(left_wrench_msg);

        if(!Operate_tool::switch_controller("scaled_pos_joint_traj_controller","selfdefined_trajectory_controller", "left_robot")){
            while(1){
                ROS_ERROR("Unable to switch controllers，The robotic arm may lose control");
                ros::Duration(0.1).sleep();
            }
        }

        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
        left_mgtr->setPlannerId("PTP");

        //Open the gripper
        gripper_stop(right_gripper_client);
        gripper_ctrl(right_gripper_client,-1,-1,850,50,80,99999999,0);

        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        return result=="clamping&blocking"?"success":result;
    }
    double Robot_operation::Weight_interrupt_judge(void){
        using namespace std;
        setlocale(LC_ALL, "");
        double weight;
        while(true){
            cout << "Please enter the current balance reading(mg):" << endl;
            try{
                scanf("%lf",&weight);
                return weight;
            }catch(...){
            }
        }
    }
    std::string replaceChar(const std::string& str, char oldChar, char newChar) {
        std::string result = str;
        for (char& c : result) {
            if (c == oldChar) {
                c = newChar;
            }
        }
        return result;
    }
    void Robot_operation::Weight_reject_spoon(int spoon){
        if(spoon==3||spoon==0)return;
        char spoon_char=spoon==3?'x':(spoon+'0');
        ROS_INFO("Start putting it back%sMedicine spoon",spoon==1?"small":"big");
        Analysis_Command("A L LIN J 0.2 0.2 powder_waitout",false);
        Analysis_Command("A R PTP J 0.2 0.2 spoon_move2",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move1",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_highout",false);
        Analysis_Command(replaceChar("A R PTP J 0.3 0.3 spoon@_high",'@',spoon_char),false);//spoon+'0'
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 spoon@_mid1",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R PTP J 0.05 0.05 spoon@_mid",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.03 0.03 spoon@_low",'@',spoon_char),false);
        Analysis_Command("G 9999 550 0 90",false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 spoon@_mid",'@',spoon_char),false);
        Analysis_Command("A R PTP J 0.1 0.1 spoon_highout",false);
        ROS_INFO("Successfully put it back%sMedicine spoon",spoon==1?"small":"big");
    }void Robot_operation::Weight_use_spoon(int old_spoon, int new_spoon){
        if(old_spoon==3||new_spoon==3)return;
        char old_spoon_char=old_spoon==3?'x':(old_spoon+'0');
        char new_spoon_char=new_spoon==3?'x':(new_spoon+'0');
        ROS_INFO("Start switching%sMedicine spoon->%sMedicine spoon",old_spoon==1?"small":"big",new_spoon==1?"small":"big");
        Analysis_Command("A L LIN P 0.2 0.2 powder_waitout",false);
        Analysis_Command("A R PTP J 0.2 0.2 spoon_move2",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move1",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_highout",false);
        Analysis_Command(replaceChar("A R PTP J 0.3 0.3 spoon@_high",'@',old_spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 spoon@_mid1",'@',old_spoon_char),false);
        Analysis_Command(replaceChar("A R PTP J 0.05 0.05 spoon@_mid",'@',old_spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.03 0.03 spoon@_low",'@',old_spoon_char),false);
        Analysis_Command("G 9999 550 0 90",false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 spoon@_mid",'@',old_spoon_char),false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 spoon@_mid",'@',new_spoon_char),false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 spoon@_low",'@',new_spoon_char),false);
        Analysis_Command("G -1 0 0 90",false);
        Analysis_Command(replaceChar("A R LIN P 0.03 0.03 spoon@_mid",'@',new_spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 spoon@_high",'@',new_spoon_char),false);
        Analysis_Command("A R PTP J 0.3 0.3 spoon_highout",false);
        Analysis_Command("A L LIN P 0.2 0.2 powder_waitout",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move1",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move2",false);
        Analysis_Command("A L LIN P 0.2 0.2 powder_wait",false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 powder@_wait",'@',new_spoon_char),false);
        ROS_INFO("Successfully switched%sMedicine spoon->%sMedicine spoon",old_spoon==1?"small":"big",new_spoon==1?"small":"big");
    }
    void Robot_operation::Weight_use_spoon(int spoon){
        if(spoon==3||spoon==0)return;
        char spoon_char=spoon==3?'x':(spoon+'0');
        ROS_INFO("Get started%sMedicine spoon",spoon==1?"small":"big");
        Analysis_Command("A L LIN P 0.2 0.2 powder_waitout",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_highout",false);
        Analysis_Command("G 9999 550 0 90",false);
        Analysis_Command(replaceChar("A R PTP J 0.3 0.3 spoon@_mid",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 spoon@_low",'@',spoon_char),false);
        Analysis_Command("G -1 0 0 90",false);
        Analysis_Command(replaceChar("A R LIN P 0.03 0.03 spoon@_mid",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 spoon@_high",'@',spoon_char),false);
        Analysis_Command("A R PTP J 0.3 0.3 spoon_highout",false);
        Analysis_Command("A L LIN P 0.2 0.2 powder_waitout",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move1",false);
        Analysis_Command("A R PTP J 0.7 0.7 spoon_move2",false);
        Analysis_Command("A L LIN P 0.2 0.2 powder_wait",false);
        Analysis_Command(replaceChar("A R PTP J 0.1 0.1 powder@_wait",'@',spoon_char),false);
        ROS_INFO("Successfully used%sMedicine spoon",spoon==1?"small":"big");
    }
    void Robot_operation::Weight_take_powder(int spoon){
        char spoon_char=spoon==3?'x':(spoon+'0');
        ROS_INFO("Start taking powder");
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 powder@_wait",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.05 0.05 powder@_in",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.1 0.1 powder@_take0",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.01 0.01 powder@_level0",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.04 0.04 powder@_level1",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.03 0.03 powder@_in",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.02 0.02 powder@_wait",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.02 0.02 powder@_out",'@',spoon_char),false);
        Analysis_Command(replaceChar("A L LIN J 0.1 0.02 powder_wait_path",'@',spoon_char),false);
        Analysis_Command(replaceChar("A L LIN J 0.1 0.01 reset",'@',spoon_char),false);
        ROS_INFO("Successfully removed powder");
    }
    void Robot_operation::Weight_putback_powder(int spoon){
        char spoon_char=spoon==3?'x':(spoon+'0');
        ROS_INFO("Start putting it backpowder");
        Analysis_Command(replaceChar("A L LIN J 0.1 0.01 powder_wait_path",'@',spoon_char),false);
        Analysis_Command(replaceChar("A L LIN J 0.1 0.02 powder_wait",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.02 0.02 powder@_out",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.02 0.02 powder@_wait",'@',spoon_char),false);
        Analysis_Command(replaceChar("A R LIN P 0.02 0.02 powder@_in",'@',spoon_char),false);
        Analysis_Command("g -1 -1 -1 -1 -1 50",false);
        Analysis_Command("G 9999 -1 0 -65",false);
        Analysis_Command("G 9999 -1 0 25",false);
        Analysis_Command("G 9999 -1 0 160",false);
        // Analysis_Command("G 9999 -1 0 160",false);
        Analysis_Command(replaceChar("A R LIN P 0.1 0.1 powder@_wait",'@',spoon_char),false);
        ROS_INFO("End and put back the powder");
    }
    std::string Robot_operation::Weight_fuzzy_shake(const std::vector<std::any>& args){//Automatically switch spoons
        ROS_INFO("fuzzy control：Start weighing the powder");
        double torr=20;
        FuzzyParams spoonParams;
        bool continue_flag=false,force_full_flag=false;
        int major_round=0,take_round=1,shake_round=1,spoon_id=0;
        double run_time=0.05,targ_weight=0;
        Motion_tool::TwistMoveMsg msg,origin_msg;
        int count=0;
        double last_weight=0,curr_weight=0,step_radius=0,total_radius=0;
        double val;
        for(int i=0;i<(int)args.size();i++){
            try {
                val=std::any_cast<double>(args[i]);
                msg[count++]=val;
            } catch (...) {
                std::string str=std::any_cast<std::string>(args[i]);
                try{
                    val=stod(str.substr(1));
                }catch(...){
                    printf("illegal val: %s\n",str.c_str());
                    continue;
                }
                switch(str[0]){
                    case 'R'://Total weighing cycles
                        major_round=(int)val;
                        printf("major round=%d\n",major_round);
                        break;
                    case 'r'://Shake each spoon several times
                        take_round=(int)val;
                        printf("take round=%d\n",take_round);
                        break;
                    case 's'://sShake the reagent bottle behind the spoon
                        shake_round=(int)val;
                        printf("shake round=%d\n",shake_round);
                    case 'T':case 't'://timing jitter 
                        run_time=val;
                        printf("run time=%.3lf\n",run_time);
                        break;
                    case 'W':case 'w'://Target quality
                        targ_weight=val;
                        printf("target weight=%.1lf\n",targ_weight);
                        break;
                    case 'C':case 'c':
                        continue_flag=true;
                    case 'F':
                        force_full_flag=true;
                    default:
                        msg.add(str);
                        break;
                }
            }
        }
        double e,ec,a;//fuzzy controlinput
        last_weight=Balance_read(balance_client);
        e=targ_weight-curr_weight,ec=curr_weight-last_weight;
        if(e<torr){
            ROS_INFO("No weighing required: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
            return std::to_string(curr_weight);
        }
        char str[3];
        origin_msg=msg;
        bool full_flag=true;
        int shake_count=0,take_count=take_round;
        double full_weight=spoonParams.full,last_full_weight=spoonParams.full;
        int msg_flag=1;
        double sum=0;
        // Analysis_Command("o Weight_shake_bottle R4 T0.2 x-0.02",false);
        while(major_round--){
            curr_weight=Balance_read(balance_client);
            if(full_flag&&spoon_id){
                double real_full_weight=curr_weight-last_weight;
                if(real_full_weight<full_weight){
                    // ROS_INFO("updatemoresmallof%sSpoon loading amount=%.1lf\n",spoon_id==1?"small":"big",real_full_weight);
                    ROS_INFO("this roundupdatemoresmallofMedicine spooncharge weight=%.1lf\n",real_full_weight);
                    full_weight=real_full_weight;
                }
                if(shake_count==1||last_full_weight==0){
                    ROS_INFO("Global update of smaller medication key dosage=%.1lf\n",last_full_weight);
                    last_full_weight=real_full_weight;
                }
                last_weight=curr_weight;
            }
            full_flag=false;
            // printf("last=%.1lf curr=%.1lf\n",last_weight,curr_weight);
            e=targ_weight-curr_weight,ec=curr_weight-last_weight;
            last_weight=curr_weight;
            // if(e>bigSpoonParams.e[1]){//Usebigspoon
            //     if(spoon_id==0)Weight_use_spoon(2);
            //     else if(spoon_id==1)Weight_use_spoon(1,2);
            //     if(spoon_id!=2)spoonParams=bigSpoonParams;
            //     spoon_id=2;
            // }else if(e>1){//Use a small spoon
            //     if(spoon_id==0)Weight_use_spoon(1);
            //     else if(spoon_id==2)Weight_use_spoon(2,1);
            //     if(spoon_id!=1)spoonParams=smallSpoonParams;
            //     spoon_id=1;
            // }
            if(e>=torr){//Use a spoon3
                // if(spoon_id==0)Weight_use_spoon(spoon_id=3);
                if(!spoon_id){
                    spoonParams=bigSpoonParams;
                    full_weight=spoonParams.full;
                    last_full_weight=spoonParams.full;
                    ec=spoonParams.ec[1];
                    spoon_id=3;
                }
            }else{
                ROS_INFO("No weighing required: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
                Weight_putback_powder(spoon_id);
                Weight_reject_spoon(spoon_id);
                return std::to_string(curr_weight);
            }
            if(take_count>=take_round){
                Weight_take_powder(spoon_id);
                take_count=0;
                msg_flag=1;
            }
            take_count++;
            if(e<torr){
                ROS_INFO("Weighing successful: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
                Weight_putback_powder(spoon_id);
                Weight_reject_spoon(spoon_id);
                return std::to_string(curr_weight);
            }
            if(e>full_weight*1.1||force_full_flag){
                ROS_INFO("Pour the whole spoon e=%.1lf full=%.1lf",e,full_weight);
                gripper_ctrl(right_gripper_client,-1,-1,-1,-1,5,340,0);
                gripper_ctrl(right_gripper_client,-1,-1,-1,-1,-1,160,0);
                Analysis_Command("A L LIN J 0.1 0.02 reset",false);
                Analysis_Command("A L LIN J 0.1 0.01 powder_wait_path",false);
                Analysis_Command("A L LIN J 0.1 0.02 powder_wait",false);
                full_flag=true;
                take_count=take_round;
                shake_count++;
            }else{
                if(e<torr)a=spoonParams.a_min;
                else a=std::max(calculate_A(e,ec,spoonParams),spoonParams.a_min);
                if(sum+a>spoonParams.a[2]||sum+a<-spoonParams.a[2])a=-a;
                sum+=a;
                // msg=Motion_tool::TwistMoveMsg();
                // if(a<1)msg.z()=-a/1000.0;
                // else msg.y()=-a/1000.0;
                msg=origin_msg.set(a/1000.0);
                ROS_INFO("data:e=%.1lf ec=%.1lf a=%.2lf div=%.2lf full=%.2f",e,ec,a,sum,full_weight);
                if(continue_flag){
                    do{
                        printf("continue or not? Y/N:");
                        scanf("%s",str);
                        if(str[0]=='n'||str[0]=='N'){
                            Weight_putback_powder(spoon_id);
                            return std::to_string(curr_weight);
                        }
                    }while(str[0]!='Y'&&str[0]!='y');
                }
                // msg.print();
                Motion_tool::twist_publish(nh_,right_twist_pub,msg,run_time);
                curr_weight=Balance_read(balance_client);
                if(curr_weight>=targ_weight-torr){
                    ROS_INFO("Weighing successful: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
                    Weight_putback_powder(spoon_id);
                    Weight_reject_spoon(spoon_id);
                    return std::to_string(curr_weight);
                }
            }
            // msg_flag*=-1;
            if(take_count>=take_round){
                if(!full_flag){
                    if(curr_weight>=targ_weight-torr){
                        ROS_INFO("Weighing successful: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
                        Weight_putback_powder(spoon_id);
                        Weight_reject_spoon(spoon_id);
                        return std::to_string(curr_weight);
                    }
                    Weight_putback_powder(spoon_id);
                }
                sum=0;
                shake_count++;
                if(shake_count>=shake_round){
                    // Analysis_Command("o Weight_shake_bottle R4 T0.2 x-0.02",false);
                    shake_count=0;
                    full_weight=last_full_weight;
                }
            }
        }ROS_INFO("Not exceeding the specified number of timesWeighing successful: target=%.1lf current=%.1lf exceed=%.1lf\n",targ_weight,curr_weight,curr_weight-targ_weight);
        Weight_reject_spoon(spoon_id);
        return std::to_string(curr_weight);
    }
    double Robot_operation::triangular_membership(double x, double p[]) {
        if (x <= p[0])return 0;
        if (x <= p[1])return (x - p[0]) / (p[1] - p[0]);
        if (x <= p[2])return 1;
        if (x <= p[3])return (p[3] - x) / (p[3] - p[2]);
        return 0;
    }

    // fuzzy inferencecalculateVibration amplitudedegreeA
    double Robot_operation::calculate_A(double E, double E_c, FuzzyParams& params) {
        params.limit(E,E_c);//restrictionEandEcScope of
        // definitionEThe degree of membershipfunction parameters
        double PINF=10,NINF=-10;
        // double e_ZO=1,e_NS=100,e_NB=2000;
        // double ec_ZO=1,ec_PS=30,ec_PB=200;
        // double a_Z=0.1,a_S=1,a_B=10;

        double E_ZO=std::log10(params.e[0]),E_NS=std::log10(params.e[1]),E_NB=std::log10(params.e[2]);
        double NB_E_params[4] = {E_NS,E_NB,PINF,PINF};
        double NS_E_params[4] = {E_ZO,E_NS,E_NS,E_NB};
        double ZO_E_params[4] = {NINF,NINF,E_ZO,E_NS};

        // definitionEcThe degree of membershipfunction parameters
        double Ec_ZO=std::log10(params.ec[0]),Ec_PS=std::log10(params.ec[1]),Ec_PB=std::log10(params.ec[2]);
        double ZO_Ec_params[4] = {NINF,NINF,Ec_ZO,Ec_PS};
        double PS_Ec_params[4] = {Ec_ZO,Ec_PS,Ec_PS,Ec_PB};
        double PB_Ec_params[4] = {Ec_PS,Ec_PB,PINF,PINF};

        // definitionAThe degree of membershipfunction parameters
        // double A_Z=std::log10(a_Z),A_S=std::log10(a_S),A_B=std::log10(a_B);
        // double Z_A_params[4] = {NINF,NINF,A_Z,A_S};
        // double S_A_params[4] = {A_Z,A_S,A_S,A_B};
        // double B_A_params[4] = {A_S,A_B,PINF,PINF};

        // calculateEThe degree of membership
        double NB_E = triangular_membership(std::log10(E), NB_E_params);
        double NS_E = triangular_membership(std::log10(E), NS_E_params);
        double ZO_E = triangular_membership(std::log10(E), ZO_E_params);

        // calculateEcThe degree of membership
        double ZO_Ec = triangular_membership(std::log10(E_c), ZO_Ec_params);
        double PS_Ec = triangular_membership(std::log10(E_c), PS_Ec_params);
        double PB_Ec = triangular_membership(std::log10(E_c), PB_Ec_params);

        // Fuzzy rule reasoning
        double rule1 = NB_E * ZO_Ec; // rulethen：ifEyesNBandEcyesZO，thenAyesB
        double rule2 = NB_E * PS_Ec; // rulethen：ifEyesNBandEcyesPS，thenAyesB
        double rule3 = NB_E * PB_Ec; // rulethen：ifEyesNBandEcyesPB，thenAyesS
        double rule4 = NS_E * ZO_Ec; // rulethen：ifEyesNSandEcyesZO，thenAyesS
        double rule5 = NS_E * PS_Ec; // rulethen：ifEyesNSandEcyesPS，thenAyesS
        double rule6 = NS_E * PB_Ec; // rulethen：ifEyesNSandEcyesPB，thenAyesZ
        double rule7 = ZO_E * ZO_Ec; // rulethen：ifEyesZOandEcyesZO，thenAyesZ
        double rule8 = ZO_E * PS_Ec; // rulethen：ifEyesZOandEcyesPS，thenAyesZ
        double rule9 = ZO_E * PB_Ec; // rulethen：ifEyesZOandEcyesPB，thenAyesZ

        // Calculate the final using the center of gravity methodAvalue
        double numerator =  rule1 * params.a[2] + 
                            rule2 * params.a[2] + 
                            rule3 * params.a[1] + 
                            rule4 * params.a[1] + 
                            rule5 * params.a[1] + 
                            rule6 * params.a[0] + 
                            rule7 * params.a[0] + 
                            rule8 * params.a[0] + 
                            rule9 * params.a[0];
        double denominator = rule1 + rule2 + rule3 + rule4 + rule5 + rule6 + rule7 + rule8 + rule9;

        if (denominator > 0)return numerator / denominator;
        return 0;
    }
    std::string Robot_operation::Weight_shake_bottle(const std::vector<std::any>& args){//Inch reference value，
        ROS_INFO("Start shaking the reagent bottle");
        int major_round=0;
        double run_time=0.05;
        Motion_tool::TwistMoveMsg msg;
        int count=0;
        double val;
        for(int i=0;i<(int)args.size();i++){
            try {
                val=std::any_cast<double>(args[i]);
                msg[count++]=val;
            } catch (...) {
                std::string str=std::any_cast<std::string>(args[i]);
                try{
                    val=stod(str.substr(1));
                }catch(...){
                    printf("illegal val: %s\n",str.c_str());
                    continue;
                }
                switch(str[0]){
                    case 'R':
                        major_round=(int)val;
                        // printf("major round=%d\n",major_round);
                        break;
                    case 'T':case 't':
                        run_time=val;
                        // printf("run time=%.3lf\n",run_time);
                        break;
                    default:
                        msg.add(str);
                        break;
                }
            }
        }
        Analysis_Command("A L LIN J 0.1 0.1 powder_wait",false);
        Analysis_Command("A L LIN J 0.3 0.3 powder_shake_wait1",false);
        Analysis_Command("A L LIN J 0.7 0.7 powder_shake_wait2",false);
        while(major_round--){
            Motion_tool::twist_publish(nh_,left_twist_pub,msg,run_time);
            msg*=-1;
        }
        Analysis_Command("A L LIN J 0.7 0.7 powder_shake_wait2",false);
        Analysis_Command("A L LIN J 0.7 0.7 powder_shake_wait1",false);
        Analysis_Command("A L LIN J 0.1 0.1 powder_wait",false);
        ROS_INFO("End shaking the reagent bottle");
        return "";
    }
    
    //handoverflagupdate
    std::string Robot_operation::Screw_rejectRGIC(void){Screw_rejecttool(EndTool::RGIC);return "";}
    std::string Robot_operation::Screw_useRGIC(void){Screw_usetool(EndTool::RGIC);return "";}
    std::string Robot_operation::Screw_rejectADP(void){Screw_rejecttool(EndTool::Pipette);return "";}
    std::string Robot_operation::Screw_useADP(void){Screw_usetool(EndTool::Pipette);return "";}
    void Robot_operation::Screw_rejecttool(EndTool cur_tool)
    {
        // ----Add transition action
        switch (cur_tool)
        {
        //---Release the current end effector
        case RGIC:
            // ----PlacementRGICThe action of the robotic arm
            R_RGIC_flag = false;
            // ----PlacementRGICThe action of the robotic arm
            break;
        case Pipette:
            // ----PlacementRGICThe action of the robotic arm
            ADP1000_flag = false;
            // ----PlacementPipetteThe action of the robotic arm
            break;

        default:
            ROS_ERROR("These is not the name of current_tool!");
            break;
        }
    }
    
    void Robot_operation::Screw_usetool(EndTool tar_tool)
    {
        // ----Add transition action
        switch (tar_tool)
        {
        //---Link target end effector
        case RGIC:
            // ----Add PickRGICThe action of the robotic arm
            R_RGIC_flag = true;
            // ----Add PickRGICThe action of the robotic arm
            break;
        case Pipette:
            // ----Add PickRGICThe action of the robotic arm
            ADP1000_flag = true;
            // ----Add PickPipetteThe action of the robotic arm
            break;
        default:
            ROS_ERROR("These is not the name of target_tool!");
            break;
        }
    }
    std::string Robot_operation::Screw_float(void){
        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        left_mgtr->setPlanningPipelineId("ompl");
        left_mgtr->setPlannerId("RRTConnect");
        Operate_tool::switch_controller("selfdefined_trajectory_controller",
                                        "scaled_pos_joint_traj_controller", "left_robot");
        
        ros::Duration(0.5).sleep();
        geometry_msgs::WrenchStamped left_wrench_msg;
        while(1){
            double x,y,z;
            printf("float x,y,z=");
            scanf("%lf%lf%lf",&x,&y,&z);
            // Target torque value
            left_wrench_msg.header.stamp = ros::Time::now();
            left_wrench_msg.header.frame_id = "left_tool0";
            left_wrench_msg.wrench.force.x = x; // Setting Powerxportion
            left_wrench_msg.wrench.force.y = y; // Setting Poweryportion,Negative values indicate downward pressure
            left_wrench_msg.wrench.force.z = z; // Setting Powerzportion
            left_wrench_msg.wrench.torque.x = 0.0;
            left_wrench_msg.wrench.torque.y = 0.0;
            left_wrench_msg.wrench.torque.z = 0.0;
            left_wrench_pub.publish(left_wrench_msg);
            //Bottle cap touch detection
            printf("Do you want to Stop(s) or Close(c):");
            char res[3];
            scanf("%s",res);
            left_wrench_msg.header.stamp = ros::Time::now();
            left_wrench_msg.header.frame_id = "left_tool0";
            left_wrench_msg.wrench.force.x = 0; // Setting Powerxportion
            left_wrench_msg.wrench.force.y = 0; // Setting Poweryportion,Negative values indicate downward pressure
            left_wrench_msg.wrench.force.z = 0; // Setting Powerzportion
            left_wrench_msg.wrench.torque.x = 0.0;
            left_wrench_msg.wrench.torque.y = 0.0;
            left_wrench_msg.wrench.torque.z = 0.0;
            left_wrench_pub.publish(left_wrench_msg);
            Operate_tool::switch_controller("selfdefined_trajectory_controller",
                                            "scaled_pos_joint_traj_controller", "left_robot");
            if(res[0]=='c')break;
            ros::Duration(0.5).sleep();
            // ros::Duration(0.5).sleep();
        }

        left_wrench_msg.header.stamp = ros::Time::now();
        left_wrench_msg.header.frame_id = "left_tool0";
        left_wrench_msg.wrench.force.x = 0.0; // Setting Powerxportion
        left_wrench_msg.wrench.force.y = 0.0; // Setting Poweryportion
        left_wrench_msg.wrench.force.z = 0.0; // Setting Powerzportion
        left_wrench_msg.wrench.torque.x = 0.0;
        left_wrench_msg.wrench.torque.y = 0.0;
        left_wrench_msg.wrench.torque.z = 0.0;
        left_wrench_pub.publish(left_wrench_msg);
        ros::Duration(0.5).sleep();

        Operate_tool::switch_controller("scaled_pos_joint_traj_controller",
                                        "selfdefined_trajectory_controller", "left_robot");

        left_mgtr->setPlanningPipelineId("pilz_industrial_motion_planner");
        left_mgtr->setPlannerId("PTP");
        if(Dashboard_flag){
            left_dbptr->zeroFtSensor();
        }
        return "";
    }
    template<class T>T Robot_operation::my_min(const T& x,const T& y){return x>y?y:x;}
}