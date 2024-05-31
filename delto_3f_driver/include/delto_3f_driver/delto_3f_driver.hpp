#include <vector>
#include <string>

 #include <ros/ros.h>
 

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <thread>
#include <mutex>
#include <vector>


#include "delto_3f_TCP.hpp"
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryServer;
  
class Delto3FDriver
{
    public:
    Delto3FDriver();
    virtual ~Delto3FDriver() = default;

    private:
    void connect();
    std::vector<double> get_position_rad();
    std::vector<double> get_position_deg();

    void target_joint_callback(const boost::shared_ptr<const std_msgs::Float32MultiArray>& msg);
    void joint_state_publisher();
    void timer_callback();

    void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void goalCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
    void cancelCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);


    
    private:

    ros::Publisher joint_state_pub;
    ros::Subscriber grasp_sub;
    ros::Subscriber grasp_mode_sub;
    ros::Subscriber target_joint_sub;

    //trajectrory for moveit 

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryServer;
  
    boost::shared_ptr<FollowJointTrajectoryServer> follow_joint_trajectory_server_;
    
    ros::Timer joint_state_timer;

    std::unique_ptr<Delto3F_TCP> delto_client;
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
    double publish_rate;
    int joint_publish_count;
    // float publish_rate;
    sensor_msgs::JointState joint_state_;
};