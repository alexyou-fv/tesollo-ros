#include "delto_2f_driver/delto_2f_driver.hpp"

Delto2FDriver::Delto2FDriver() : publish_rate(100.0)
{
    ros::NodeHandle nh_;
    nh_.param<std::string>("ip", delto_ip, std::string("192.168.0.210"));
    nh_.param<int>("port", delto_port, 10000);
    nh_.param<int>("slaveID", slaveID, 1);
    nh_.param<int>("dummy", false);

    double kp = 1.0;
    double kd = 5.0;

    double target_joint_state = 0;
    double current_joint_state = 0;

    // publisher and subscriber
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/gripper/joint_states", 10);
    target_joint_sub = nh_.subscribe<std_msgs::Float32>("/gripper/target_position", 10, boost::bind(&Delto2FDriver::target_position_callback, this, _1));
    joint_state_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&Delto2FDriver::timer_callback, this));

    // Initialize the action server
    follow_joint_trajectory_server_ = boost::make_shared<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
        "follow_joint_trajectory",
        boost::bind(&Delto2FDriver::execute_callback, this, _1),
        false); // don't start the server yet
    // Start the action server
    follow_joint_trajectory_server_->start();

    auto joint_state = sensor_msgs::JointState();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"Distance"};

    std::cout << "delto_ip : " << delto_ip << std::endl;
    std::cout << "delto_port : " << delto_port << std::endl;

    delto_client = std::make_unique<Delto2F_TCP>(delto_ip, delto_port);

    connect();
}

void Delto2FDriver::connect()
{
    delto_client->connect();
}

void Delto2FDriver::joint_state_publisher()
{
    // auto joint_state_ = sensor_msgs::JointState();
    joint_state_.header.stamp = ros::Time::now();
    joint_state_.name = {"Distance"};
    joint_state_.position = std::vector<double>{current_joint_state};
    joint_state_pub.publish(joint_state_);
}

void Delto2FDriver::timer_callback()
{
    auto positions = delto_client->get_position();


    current_joint_state = positions;
    joint_state_publisher();
    
}

void Delto2FDriver::target_position_callback(const boost::shared_ptr<const std_msgs::Float32> &msg)
{

    auto target_data = msg->data;
    

    delto_client->set_postion(target_data);
}
void Delto2FDriver::execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    // this->execute_callback_test(goal);
    // return;
}

void Delto2FDriver::goalCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    // ROS_INFO("Received goal request with %lu points", gh.getGoal()->trajectory.points.size());
    // gh.setAccepted();
}

void Delto2FDriver::cancelCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    // ROS_INFO("Received request to cancel goal");
    // gh.setCanceled();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delto_2F_driver");
    boost::shared_ptr<Delto2FDriver> node(new Delto2FDriver());

    ros::AsyncSpinner spinner(4); // 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}