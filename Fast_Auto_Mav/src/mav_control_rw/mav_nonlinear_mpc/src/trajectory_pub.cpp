#include <thread>
#include <chrono>
#include <cmath>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

struct Pose
{
    double x, y, z;
    double qx, qy, qz, qw;
};
std::vector<Pose> poses;

mavros_msgs::State current_state;
nav_msgs::Odometry odom;

ros::Time curr_time;
ros::Time cmd_time;
Eigen::Vector3d curr_position;
Eigen::Vector3d cmd_position;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_locus_example");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");
    ros::Publisher trajectory_pub =
            nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    "/sim/command/trajectory", 10);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, odom_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    ROS_INFO("Started circle locus example.");

    //读取csv文件
    std::ifstream file("/home/c2214/Downloads/trajectory_out.csv");
    if (!file.is_open())
        ROS_ERROR("Could not open file");
    std::string line;
    while(std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        Pose pose;
        std::istringstream ss(line);
        std::string col;
        std::getline(ss,col,',');
        pose.x = std::stod(col);
        std::getline(ss,col,',');
        pose.y = std::stod(col);
        std::getline(ss,col,',');
        pose.z = std::stod(col);
        std::getline(ss,col,',');
        pose.qx = std::stod(col);
        std::getline(ss,col,',');
        pose.qy = std::stod(col);
        std::getline(ss,col,',');
        pose.qz = std::stod(col);
        std::getline(ss,col,',');
        pose.qw = std::stod(col);
        poses.push_back(pose);
    }
    file.close();
    std::cout << "Read " << poses.size() << " poses" << std::endl;
//    std::cout << poses[0].x << " " << poses[0].y << " " << poses[0].z << " " << poses[0].qx << " " << poses[0].qy << " " << poses[0].qz << " " << poses[0].qw << std::endl;
//    std_srvs::Empty srv;
//    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
//    unsigned int i = 0;
//
//    // Trying to unpause Gazebo for 10 seconds.
//    while (i <= 10 && !unpaused) {
//        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
//        ++i;
//    }
//
//    if (!unpaused) {
//        ROS_FATAL("Could not wake up Gazebo.");
//        return -1;
//    } else {
//        ROS_INFO("Unpaused the Gazebo simulation.");
//    }
    ros::Rate rate(100);
    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    geometry_msgs::PoseStamped pos_msg;


    pos_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.stamp = ros::Time::now();
    double begin_t = 0;
    double t = 0;
    double angle = 0;

    // Default desired position and yaw.
    Eigen::Vector3d desired_position(0.0, 0.0, 3.0);
    double desired_yaw = 0.0;

//    // Overwrite defaults if set as node parameters.
//    nh_private.param("x", desired_position.x(), desired_position.x());
//    nh_private.param("y", desired_position.y(), desired_position.y());
//    nh_private.param("z", desired_position.z(), desired_position.z());
//    nh_private.param("yaw", desired_yaw, desired_yaw);

    size_t current_index = 0;
    int first_flag = 1;
    while (ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    begin_t = ros::Time::now().toSec();
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        Eigen::Quaterniond q_des_0;
        Eigen::Quaterniond q_des;
        Eigen::Vector3d eular_angles_0;
        Eigen::Vector3d eular_angles;

        if(current_state.mode != "OFFBOARD" || !current_state.armed || (ros::Time::now().toSec()-begin_t) < 10){
            if(first_flag)
            {
            desired_position.x() = odom.pose.pose.position.x;
            desired_position.y() = odom.pose.pose.position.y;
            desired_position.z() = poses[0].z;
            q_des_0 = Eigen::Quaterniond(poses[0].qw, poses[0].qx, poses[0].qy, poses[0].qz);
            mav_msgs::getEulerAnglesFromQuaternion(q_des_0,&eular_angles_0);
            desired_yaw = 0;
                first_flag = 0;
            }

//            desired_position.x() = poses[0].x;
//            desired_position.y() = poses[0].y;
//            desired_position.z() = poses[0].z;
//            desired_yaw = atan2(2 * (poses[0].qw * poses[0].qz + poses[0].qx * poses[0].qy),
//                                1 - 2 * (poses[0].qy * poses[0].qy + poses[0].qz * poses[0].qz));
        }
        else if((ros::Time::now().toSec()-begin_t) < 20)
        {
            desired_position.x() = poses[0].x;
            desired_position.y() = poses[0].y;
            desired_position.z() = poses[0].z;
            desired_yaw = 0;
        }
        else if(current_index < poses.size())
        {
//            desired_position.x() = poses[0].x;
//            desired_position.y() = poses[0].y;
//            desired_position.z() = poses[0].z;
//            desired_yaw = atan2(2 * (poses[0].qw * poses[0].qz + poses[0].qx * poses[0].qy),
//                                1 - 2 * (poses[0].qy * poses[0].qy + poses[0].qz * poses[0].qz));
            q_des = Eigen::Quaterniond(poses[current_index].qw, poses[current_index].qx, poses[current_index].qy, poses[current_index].qz);
            mav_msgs::getEulerAnglesFromQuaternion(q_des,&eular_angles);
            desired_position.x() = poses[current_index].x;
            desired_position.y() = poses[current_index].y;
            desired_position.z() = poses[current_index].z;
            desired_yaw = 0;
//            desired_yaw = atan2(2*(poses[current_index].qw*poses[current_index].qz + poses[current_index].qx*poses[current_index].qy),
//                                1-2*(poses[current_index].qy*poses[current_index].qy + poses[current_index].qz*poses[current_index].qz));

            current_index++;

            std::ofstream cmd_traj;
            std::ofstream curr_traj;
            cmd_time = ros::Time::now();
            curr_time = ros::Time::now();
            cmd_position[0] = desired_position.x();
            cmd_position[1] = desired_position.y();
            cmd_position[2] = desired_position.z();
            curr_position[0] = odom.pose.pose.position.x;
            curr_position[1] = odom.pose.pose.position.y;
            curr_position[2] = odom.pose.pose.position.z;
            cmd_traj.open("/home/c2214/Projects/Fast_Auto_Mav/cmd_traj.tum", std::ios::app);
            cmd_traj << cmd_time << " " << cmd_position[0] << " " << cmd_position[1] << " " << cmd_position[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
            curr_traj.open("/home/c2214/Projects/Fast_Auto_Mav/curr_traj.tum", std::ios::app);
            curr_traj << curr_time << " " << curr_position[0] << " " << curr_position[1] << " " << curr_position[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
        }


//            t = ros::Time::now().toSec() - begin_t;
//            angle = fmod(2 * M_PI / 4 * t , 2 * M_PI); // 10s飞一圈
//            desired_position.x() = 5 * cos(angle); // 圆的半径是1
//            desired_position.y() = 5 * sin(angle);
//            desired_position.z() = 5;

            //desired_yaw = fmod(angle + M_PI/2 , 2 * M_PI);
            //desired_yaw = M_PI/2;
//        pos_msg.pose.position.x = desired_position.x();
//        pos_msg.pose.position.y = desired_position.y();
//        pos_msg.pose.position.z = desired_position.z();

            trajectory_msg.header.stamp = ros::Time::now();
            std::cout << desired_position.transpose() <<"  "<< desired_yaw << std::endl;
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                    desired_position, desired_yaw, &trajectory_msg);

            trajectory_pub.publish(trajectory_msg);
            ROS_INFO(" RUNING ");


//        t = ros::Time::now().toSec() - begin_t;
//        static int count0 = 0;

        //desired_yaw = M_PI/2;
//        pos_msg.pose.position.x = desired_position.x();
//        pos_msg.pose.position.y = desired_position.y();
//        pos_msg.pose.position.z = desired_position.z();


        //pos_pub.publish(pos_msg);
        rate.sleep();
        ros::spinOnce();

    }

    return 0;
}
