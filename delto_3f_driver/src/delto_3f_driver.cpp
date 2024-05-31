#include "delto_3f_driver/delto_3f_driver.hpp"

Delto3FDriver::Delto3FDriver() : publish_rate(100.0)
{
    ros::NodeHandle nh_;
    nh_.param<std::string>("ip", delto_ip, std::string("192.168.0.210"));
    nh_.param<int>("port", delto_port, 10000);
    nh_.param<int>("slaveID", slaveID, 1);
    nh_.param<int>("dummy", false);

    kp = std::vector<double>(12, 1.0);
    kd = std::vector<double>(12, 5.0);

    target_joint_state = std::vector<double>(12, 0.0);
    current_joint_state = std::vector<double>(12, 0.0);

    // publisher and subscriber
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/gripper/joint_states", 10);
    target_joint_sub = nh_.subscribe<std_msgs::Float32MultiArray>("/gripper/target_joint_deg", 10, boost::bind(&Delto3FDriver::target_joint_callback, this, _1));
    joint_state_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&Delto3FDriver::timer_callback, this));

    // Initialize the action server
    follow_joint_trajectory_server_ = boost::make_shared<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
        "follow_joint_trajectory",
        boost::bind(&Delto3FDriver::execute_callback, this, _1),
        false); // don't start the server yet
    // Start the action server
    follow_joint_trajectory_server_->start();

    auto joint_state = sensor_msgs::JointState();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"F1M1", "F1M2", "F1M3", "F1M4",
                        "F2M1", "F2M2", "F2M3", "F2M4",
                        "F3M1", "F3M2", "F3M3", "F3M4"};

    std::cout << "delto_ip : " << delto_ip << std::endl;
    std::cout << "delto_port : " << delto_port << std::endl;

    delto_client = std::make_unique<Delto3F_TCP>(delto_ip, delto_port);

    connect();
}

void Delto3FDriver::connect()
{
    delto_client->connect();
}

void Delto3FDriver::joint_state_publisher()
{
    // auto joint_state_ = sensor_msgs::JointState();
    joint_state_.header.stamp = ros::Time::now();
    joint_state_.name = {"F1M1", "F1M2", "F1M3", "F1M4",
                         "F2M1", "F2M2", "F2M3", "F2M4",
                         "F3M1", "F3M2", "F3M3", "F3M4"};
    joint_state_.position = current_joint_state;
    joint_state_pub.publish(joint_state_);
}

void Delto3FDriver::timer_callback()
{
    auto positions = delto_client->get_position_rad();

    if (positions.size() == 12)
    {
        current_joint_state = positions;
        joint_state_publisher();
    }
}

void Delto3FDriver::target_joint_callback(const boost::shared_ptr<const std_msgs::Float32MultiArray> &msg)
{

    std::vector<double> target_data;
    target_data.resize(12);
    for (int i = 0; i < 12; i++)
    {
        target_data[i] = msg->data[i];
    }

    delto_client->set_postion_deg(target_data);
}
void Delto3FDriver::execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    // this->execute_callback_test(goal);
    // return;
}

void Delto3FDriver::goalCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    // ROS_INFO("Received goal request with %lu points", gh.getGoal()->trajectory.points.size());
    // gh.setAccepted();
}

void Delto3FDriver::cancelCB(actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    // ROS_INFO("Received request to cancel goal");
    // gh.setCanceled();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delto_3f_driver");
    boost::shared_ptr<Delto3FDriver> node(new Delto3FDriver());

    ros::AsyncSpinner spinner(4); // 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}