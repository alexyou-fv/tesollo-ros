#include "delto_3f_driver/delto_3f_external_driver.hpp"


    DeltoExternalDriver::DeltoExternalDriver() : publish_rate(500.0)
{

    ros::NodeHandle nh_;
    nh_.param<std::string>("ip", delto_ip, std::string("192.168.0.210"));
    nh_.param<int>("port", delto_port, 10000);
    nh_.param<int>("slaveID",slaveID, 1);
    nh_.param<int>("dummy", false);

    kp = std::vector<double>(12, 1.0);
    kd = std::vector<double>(12, 5.0);

    target_joint_state = std::vector<double>(12, 0.0);
    current_joint_state = std::vector<double>(12, 0.0);
    joint_dot = std::vector<double>(12, 0.0);
    pre_joint_state = std::vector<double>(12, 0.0);


    
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/gripper/joint_states", 10);
    
    target_joint_deg_sub = nh_.subscribe<std_msgs::Float32MultiArray>("gripper/target_joint_deg", 100, boost::bind(&DeltoExternalDriver::targetjoint_deg_callback, this, _1));
    target_joint_rad_sub = nh_.subscribe<std_msgs::Float32MultiArray>("gripper/target_joint_rad", 100, boost::bind(&DeltoExternalDriver::targetjoint_rad_callback, this, _1));

    controller_timer = nh_.createTimer(ros::Duration(1.0/publish_rate), boost::bind(&DeltoExternalDriver::controller_timer_callback, this));

    auto joint_state = sensor_msgs::JointState();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"F1M1", "F1M2", "F1M3", "F1M4",
                        "F2M1", "F2M2", "F2M3", "F2M4",
                        "F3M1", "F3M2", "F3M3", "F3M4"};

    std::cout << "delto_ip : " << delto_ip << std::endl;
    std::cout << "delto_port : " << delto_port << std::endl;

    delto_client = std::make_unique<DeltoTCP::Communication>(delto_ip, delto_port);

    connect();

    prev_time = ros::Time::now();
}

DeltoExternalDriver::~DeltoExternalDriver() {}

void DeltoExternalDriver::connect()
{
    delto_client->connect();
}

std::vector<double> DeltoExternalDriver::get_position()
{
    return current_joint_state;
}

void DeltoExternalDriver::controller_timer_callback()
{

    
    std::lock_guard<std::mutex> lock(mutex_);

    
    DeltoRecievedData recieved_data = delto_client-> get_data();

    
    if (recieved_data.current.size() != 12 || recieved_data.joint.size() != 12)
    {
        ROS_ERROR("Recieved data size is not 12");    
        return;
    }

    electric_current_state = recieved_data.current;
    current_joint_state = recieved_data.joint;

    

    // pub joint state
    if (joint_publish_count++ > 5)
    {   
        auto joint_state = sensor_msgs::JointState();

        joint_state.header.stamp = ros::Time::now();
        joint_state.position = current_joint_state;
        joint_state_pub.publish(joint_state);

        joint_publish_count = 0;
    }

    // PD control
    for (int i = 0; i < 12; ++i)
    {
        joint_dot[i] = current_joint_state[i] - pre_joint_state[i];
        pre_joint_state = current_joint_state;
    }
    auto tq_u = JointControl(target_joint_state, current_joint_state, joint_dot, kp, kd);
    auto duty = Torque2Duty(tq_u);

    
    std::vector<int> calcduty;

    calcduty.reserve(12);

    for (int i = 0; i < 12; i++)
    {
        if (duty[i] > 70)
        {
            duty[i] = 70;
        }
        else if (duty[i] < -70)
        {
            duty[i] = -70;
        }

        calcduty[i] = int(duty[i] * 10.0);
    }

    delto_client->send_duty(calcduty);
}


void DeltoExternalDriver::targetjoint_rad_callback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    if(msg->data.size() != 12)
    {
        ROS_ERROR("Recieved data (target_joint) size is not 12");
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    target_joint_state = std::vector<double>(msg->data.begin(), msg->data.end());
}

void DeltoExternalDriver::targetjoint_deg_callback(const std_msgs::Float32MultiArrayConstPtr& msg)
{   
    if(msg->data.size() != 12)
    {
        ROS_ERROR("Recieved data (target_joint) size is not 12");

        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (int i = 0; i < 12; i++)
    {
        target_joint_state[i] = msg->data[i] * M_PI / 180.0;
    }
}


std::vector<double> DeltoExternalDriver::JointControl(std::vector<double> target_joint_state,
                                                      std::vector<double> current_joint_state,
                                                      std::vector<double> joint_dot,
                                                      std::vector<double> kp,
                                                      std::vector<double> kd)
{

    std::vector<double> tq_u(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        tq_u[i] = kp[i] * (target_joint_state[i] - current_joint_state[i]) - (kd[i] * joint_dot[i]);
    }

    return tq_u;
}

std::vector<double> DeltoExternalDriver::Torque2Duty(std::vector<double> tq_u)
{

    std::vector<double> duty(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        double v = 14.1 / 0.8 * tq_u[i];

        duty[i] = 100.0 * v / 11.1;

        if (duty[i] > 100.0)
        {
            duty[i] = 100.0;
        }
        else if (duty[i] < -100.0)
        {
            duty[i] = -100.0;
        }
    }

    return duty;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"delto_3f_driver");
    // DeltoExternalDriver driver;
    boost::shared_ptr<DeltoExternalDriver> node(new DeltoExternalDriver());

    ros::AsyncSpinner spinner(4); // 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;

}
