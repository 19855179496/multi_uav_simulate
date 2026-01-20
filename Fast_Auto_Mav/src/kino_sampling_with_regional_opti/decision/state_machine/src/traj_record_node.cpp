#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "iostream"
#include "fstream"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
ros::Time curr_time;
ros::Time cmd_time;
ros::Subscriber odom_sub;
ros::Subscriber cmd_sub;
Eigen::Vector3d curr_position;
Eigen::Vector3d cmd_position;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    Eigen::Vector3d curr(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    curr_position = curr;
}

void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    Eigen::Vector3d cmd(msg->position.x, msg->position.y, msg->position.z);
    cmd_position = cmd;
    cmd_time = ros::Time::now();
    curr_time = ros::Time::now();
    std::ofstream curr_traj;
    std::ofstream cmd_traj;
    curr_traj.open("/home/zk/Fast_Auto_Mav/curr_traj.tum", std::ios::app);
    curr_traj << curr_time << " " << curr_position[0] << " " << curr_position[1] << " " << curr_position[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
    cmd_traj.open("/home/zk/Fast_Auto_Mav/cmd_traj.tum", std::ios::app);
    cmd_traj << cmd_time << " " << cmd_position[0] << " " << cmd_position[1] << " " << cmd_position[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
}

void FilterCallback(const nav_msgs::OdometryConstPtr &msg1, const quadrotor_msgs::PositionCommand::ConstPtr &msg2)
{
    Eigen::Vector3d curr(msg1->pose.pose.position.x, msg1->pose.pose.position.y, msg1->pose.pose.position.z);
    Eigen::Vector3d cmd(msg2->position.x, msg2->position.y, msg2->position.z);
    std::ofstream curr_traj;
    std::ofstream cmd_traj;
    std::cout<< msg1->header.stamp<< "    "<< msg2->header.stamp<<std::endl;
    curr_traj.open("/home/zk/Fast_Auto_Mav/curr_traj.tum", std::ios::app);
    curr_traj << msg1->header.stamp << " " << curr[0] << " " << curr[1] << " " << curr[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
    cmd_traj.open("/home/zk/Fast_Auto_Mav/cmd_traj.tum", std::ios::app);
    cmd_traj << msg2->header.stamp << " " << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_record_node");
    ros::NodeHandle nh;

    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, odom_cb);
    // cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 100, cmd_cb);
    // message_filters::Subscriber<nav_msgs::Odometry> odom_f_sub(nh, "/Odometry", 10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_f_sub(nh, "/mavros/local_position/odom", 100);
    message_filters::Subscriber<quadrotor_msgs::PositionCommand> cmd_f_sub(nh, "/position_cmd", 100);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, quadrotor_msgs::PositionCommand> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(300), odom_f_sub, cmd_f_sub);
    sync.registerCallback(boost::bind(&FilterCallback, _1, _2));
    // imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",100,imu_cb);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
