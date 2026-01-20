#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ParamSet.h>
 
class RCEKFParamSwitcher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber rc_sub_;
    ros::ServiceClient param_client_;
    int target_channel_;          // 目标遥控器通道（如AUX5对应通道索引8）
    int last_switch_state_;       // 上一次拨杆状态（避免重复触发）
 
public:
    RCEKFParamSwitcher() : last_switch_state_(-1) {
        // 从参数服务器读取配置 
        nh_.param("target_channel", target_channel_, 8); // 默认AUX5（通道9，索引8）
        
        // 订阅遥控器信号 
        rc_sub_ = nh_.subscribe("/mavros/rc/in", 10, &RCEKFParamSwitcher::rcCallback, this);
        
        // 初始化参数设置服务客户端 
        param_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    }
 
    // 遥控器信号回调函数 
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        if (msg->channels.size()  <= target_channel_) {
            ROS_ERROR("Channel %d not available!", target_channel_);
            return;
        }
 
        // 获取目标通道的PWM值（通常范围：1000-2000）
        int pwm_value = msg->channels[target_channel_];
        int current_state = (pwm_value > 1800) ? 1 : 0; // 高位为1，低位为0 
 
        // 仅当状态变化时触发 
        if (current_state != last_switch_state_) {
            ROS_INFO("Switch state changed to: %d", current_state);
            setEKF2AidMask(current_state);
            last_switch_state_ = current_state;
        }
    }
 
    // 设置EKF2_AID_MASK参数 
    void setEKF2AidMask(int state) {
        mavros_msgs::ParamSet srv;
        srv.request.param_id  = "EKF2_AID_MASK";
        
        // 根据拨杆状态设置参数值
        if (state == 1) { // 高位：启用RTK
            srv.request.value.integer  = 1;  // 二进制00000001，仅RTK
        } else {          // 低位：恢复GPS+视觉 
            srv.request.value.integer  = 24; // 二进制00011000，GPS+视觉 
        }
 
        // 调用服务 
        if (param_client_.call(srv)) {
            ROS_INFO("Set EKF2_AID_MASK to %d: %s", 
                     srv.request.value.integer,  
                     srv.response.success  ? "Success" : "Failed");
        } else {
            ROS_ERROR("Failed to call param/set service");
        }
    }
};
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_aid_mask_switcher");
    RCEKFParamSwitcher switcher;
    ros::spin();
    return 0;
}