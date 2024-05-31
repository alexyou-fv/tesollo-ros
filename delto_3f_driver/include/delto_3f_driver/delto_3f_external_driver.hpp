#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <mutex>
#include <iostream>
// #include <asio/io_context.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "delto_external_TCP.hpp"



typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class DeltoExternalDriver {
public:
    DeltoExternalDriver();
    virtual ~DeltoExternalDriver();

private:
    void joint_state_publisher();
    void controller_timer_callback();
    void targetjoint_deg_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void targetjoint_rad_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  
    std::vector<double> Torque2Duty(std::vector<double> tq_u);
    std::vector<double> JointControl(std::vector<double> target_joint_state,
                                        std::vector<double> current_joint_state,
                                        std::vector<double> joint_dot,
                                        std::vector<double> kp,
                                        std::vector<double> kd);
                                        
    std::vector<double> get_position();
  
    ros::Publisher joint_state_pub;
    ros::Subscriber target_joint_deg_sub;
    ros::Subscriber target_joint_rad_sub;
    ros::Subscriber grasp_mode_sub;
    
    ros::Timer controller_timer;


    boost::shared_ptr<DeltoTCP::Communication> delto_client;
    
    // boost::mutex mutex_;
    // ros::NodeHandle nh_;

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
    boost::shared_ptr<Server> follow_joint_trajectory_server_;

    // boost::shared_ptr<Server> follow_joint_trajectory_server;
      // void execute_callback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    // void execute_callback_test(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // rclcpp_action::GoalResponse handle_goal(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
    // rclcpp_action::CancelResponse handle_cancel(
    //     const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    
    // void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // rclcpp::TimerBase::SharedPtr joint_state_timer;
    // boost::shared_ptr<DeltoTCP::Communication> delto_client;


    std::mutex mutex_;

    std::vector<double> kp;
    std::vector<double> kd;

    std::vector<double> current_joint_state;
    std::vector<double> electric_current_state;
    std::vector<double> target_joint_state;
    std::vector<double> joint_dot;
    std::vector<double> pre_joint_state;
    
    std::string delto_ip;
    int delto_port;
    int slaveID;
    bool dummy;
    // rclcpp::Clock::SharedPtr clock_;
    ros::Time prev_time;

    double publish_rate;
    int joint_publish_count;
    sensor_msgs::JointState joint_state;

    void connect();
};
