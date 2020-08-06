#include "rl_lib.h"
#include "rl_altitude_control.hpp"

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos_position = *msg;
    pos_current = ToEigen(local_pos_position.pose.position);
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    local_vel_current = *msg;
    ver_current = ToEigen(local_vel_current.twist.linear);
}

int start_position_check(geometry_msgs::PoseStamped pos)
{
    static uint16_t count = 0;
    float error_z = local_pos_position.pose.position.z - pos.pose.position.z;
    if (fabs(error_z) < 0.2)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    if (count > 50)
    {
        return 1;
    }
    return 0;
}

double tmp[minimum_snap_Row][minimum_snap_Col];
double time_tmp[minimum_snap_Row];
int8_t read_minimum_snap_para(void)
{
    std::ifstream openfile;
    openfile.open("/home/chasing/Documents/minimum_snap/data.txt", std::ios::in);
    if (!openfile.is_open())
    {
        ROS_ERROR("File open failed! Must check it, and repeated it!");
        return 1;
    }
    for (uint8_t i = 0; i < minimum_snap_Row; i++)
    {
        for (uint8_t j = 0; j < minimum_snap_Col; j++)
        {
            openfile >> tmp[i][j];
            _polyCoeff(i, j) = tmp[i][j];
        }
    }
    for (uint8_t i = 0; i < minimum_snap_Row; i++)
    {
        openfile >> time_tmp[i];
        _polyTime(i) = time_tmp[i];
    }
    openfile.close();
    // visWayPointTraj(_polyCoeff,_polyTime);
    // std::cout<<_polyCoeff<<std::endl<<_polyTime<<std::endl;
    return 0;
}

Eigen::Vector3d rect_position(ros::Duration time, float period, float length, float width)
{
    static Eigen::Vector3d rect_pos;
    static uint8_t rect_step = 0;
    double delta_length = length / period; // the function iteration 250hZ
    double delta_width = width / period;   // the function iteration 250hZ

    switch (rect_step)
    {
    case 0:
        rect_pos += delta_length / 250 * Eigen::Vector3d(1, 0, 0);
        if (rect_pos(0) > length)
            rect_step = 1;
        break;
    case 1:
        rect_pos += delta_width / 250 * Eigen::Vector3d(0, 1, 0);
        if (rect_pos(1) > width)
            rect_step = 2;
        break;
    case 2:
        rect_pos -= delta_length / 250 * Eigen::Vector3d(1, 0, 0);
        if (rect_pos(0) < 0)
            rect_step = 3;
        break;
    case 3:
        rect_pos -= delta_width / 250 * Eigen::Vector3d(0, 1, 0);
        if (rect_pos(1) < 0)
            rect_step = 0;
        break;
    }
    return rect_pos;
}

// #define geometric_control              //if not using geometric control , please cancel this line
int main(int argc, char **argv)
{
    // read_minimum_snap_para();
    uint8_t code_step = 1;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpointraw_attitude", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb);
    float period, length, width;
    nh.param<float>("/period", period, 10);
    nh.param<float>("/length", length, 2);
    nh.param<float>("/width", width, 2);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(250.0); //100hz

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
#ifdef geometric_control
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
#else
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
#endif

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    Eigen::Vector3d EularAngle_current;
    ros::Time Circle_begin_t;
    uint8_t Offboard_control_ = 0;
    Eigen::Vector3d rect_pos;
    while (ros::ok())
    {
        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            Offboard_control_ = 1;
            if (!start_position_check(pose) && code_step)
            {
                // code_step = 0;
                Circle_begin_t = ros::Time::now();
                local_pos_pub.publish(pose);
            }
            else
            {
                code_step = 0;
                ros::Duration time_t = ros::Time::now() - Circle_begin_t;
                rect_pos = rect_position(time_t, period, length, width);
                pose.pose.position.x = rect_pos(0);
                pose.pose.position.y = rect_pos(1);
                // pose.pose.position.z = rect_pos(2);
                local_pos_pub.publish(pose);
            }
        }
        else
        {
            Circle_begin_t = ros::Time::now();
        }
        if (!Offboard_control_)
        {
            local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
