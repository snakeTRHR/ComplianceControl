#include<ros/ros.h>
#include<iostream>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64.h>
#include<ros/time.h>
#include"trajectory_msgs/JointTrajectory.h"
#include<dynamixel_workbench_msgs/DynamixelCommand.h>
#include<cmath>
double joint_effort{};
double joint_effort_prev{};
double joint_pos_now{};
double joint_pos_prev{};
double joint_rad_diff{};
//一軸のコンプライアンス制御
class TorqueCompliance{
    public:
        TorqueCompliance(double _gain_k):gain_k(_gain_k){
        }
        double operator()(){
            //ROS_INFO("diff = %lf", joint_rad_diff);
            torque = joint_rad_diff * gain_k;
            return torque;
        }
    private:
        double gain_k;
        double torque;
};
constexpr double joint_rad_d = M_PI / 6;
constexpr int pulse_sum = 4096;
constexpr double one_pulse_rad = 2 * M_PI / 4096;
double posToRad(double pos_raw){
    //DXLの位置からradに変換
    double now_pulse = fmod(pos_raw, 4096.0);
    double now_rad = now_pulse * one_pulse_rad;
    //ROS_INFO("%lf, %lf", pos_raw, now_rad);
    return now_rad;
}
void dynamixelCallback(const sensor_msgs::JointState &model){
    joint_pos_now = model.position[0];
    //joint_rad_diff = joint_rad_d - posToRad(joint_pos_now);
    joint_rad_diff = joint_rad_d - fmod(joint_pos_now, M_PI * 2);
    ROS_INFO("%lf", joint_rad_diff * (180 / M_PI));
    //ROS_INFO("deg = %lf", posToRad(joint_pos_now) * (180 / M_PI));
}
int main(int argc, char **argv){
    ros::init(argc, argv, "compliance");
    ros::NodeHandle n;
    ros::Subscriber thetaSub = n.subscribe("/dynamixel_workbench/joint_states", 10, dynamixelCallback);
    ros::ServiceClient dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    ros::Rate loop_rate(40);
    ros::spinOnce();
    TorqueCompliance torqueControl(10);
    dynamixel_workbench_msgs::DynamixelCommand srv;
    while(ros::ok()){
        srv.request.command = "_";
        srv.request.id = 1;
        srv.request.addr_name = "Goal_Current";
        srv.request.value = torqueControl();
        dynamixel_service.call(srv);
        ros::spinOnce();
        loop_rate.sleep();
    }
}