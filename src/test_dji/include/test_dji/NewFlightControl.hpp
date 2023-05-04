#include "NewMyDataFun.h"
#include "NewMyMathFun.h"
#include "NewUtils.h"

#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1.0
#define UWB_INSTEAD_VO
//#define GPS_HEIGHT

ros::Publisher ctrl_cmd_pub;
ros::Publisher gimbal_angle_cmd_pub, gimbal_speed_cmd_pub;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient set_local_pos_reference;

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat) {
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    // ROS_INFO("Euler angle: %.2lf %.2lf %.2lf", ans.x, ans.y, ans.z);
    return ans;
}

geometry_msgs::QuaternionStamped current_atti;
geometry_msgs::Vector3 current_euler_angle;
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    current_atti.quaternion = msg->quaternion;
    current_euler_angle = toEulerAngle(current_atti.quaternion);
    // ROS_INFO("get attitude %s",
    // MyDataFun::output_str(current_euler_angle).c_str());
}
geometry_msgs::Vector3 current_gimbal_angle;
void gimbal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    current_gimbal_angle.x = msg->vector.y;
    current_gimbal_angle.y = msg->vector.x;
    current_gimbal_angle.z = msg->vector.z;
    // ROS_INFO("Gimbal %s", MyDataFun::output_str(current_gimbal_angle).c_str());
}

geometry_msgs::Vector3 current_pos_raw;
std_msgs::Float32 current_height;
void height_callback(const std_msgs::Float32::ConstPtr& msg) {
    current_height = *msg;
#ifndef GPS_HEIGHT
    current_pos_raw.z = current_height.data;
#endif
}
dji_osdk_ros::VOPosition current_vo_pos;
void vo_pos_callback(const dji_osdk_ros::VOPosition::ConstPtr& msg) {
    current_vo_pos = *msg;
    #ifndef UWB_INSTEAD_VO
    current_pos_raw.x = current_vo_pos.y;
    current_pos_raw.y = current_vo_pos.x;
    #endif
}

geometry_msgs::Vector3 current_local_pos;
void local_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
    current_local_pos.x = msg->point.x;
    current_local_pos.y = msg->point.y;
    current_local_pos.z = msg->point.z;
#ifdef GPS_HEIGHT
    current_pos_raw.z = current_local_pos.z;
#endif
}

double yaw_offset;
Point position_offset;
Point compensate_position_offset(Point _p){
    Point res;
    res.x = _p.x + position_offset.x;
    res.y = _p.y + position_offset.y;
    res.z = _p.z;
    return res;
}
Point compensate_offset(Point _p){
    Point res;
    res.x = _p.x * cos(yaw_offset) - _p.y * sin(yaw_offset) + position_offset.x;
    res.y = _p.x * sin(yaw_offset) + _p.y * cos(yaw_offset) + position_offset.y;
    res.z = _p.z;
    return res;
}
Point compensate_yaw_offset(Point _p, double _y){
    Point res;
    res.x = _p.x * cos(_y) - _p.y * sin(_y);
    res.y = _p.x * sin(_y) + _p.y * cos(_y);
    res.z = _p.z;
    return res;
}

geometry_msgs::Vector3 current_range_pos;
void range_pos_callback(const nav_msgs::Odometry::ConstPtr& msg){
    MyDataFun::set_value(current_range_pos, msg->pose.pose.position);
    current_range_pos = compensate_yaw_offset(current_range_pos, yaw_offset);
    #ifdef UWB_INSTEAD_VO
    current_pos_raw.x = current_range_pos.x;
    current_pos_raw.y = current_range_pos.y;
    #endif
}

uint8_t flight_status = 255;
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
	flight_status = msg->data;
}


uint8_t display_mode = 255;
void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg) {
	display_mode = msg->data;
}


bool EMERGENCY = false;
std::string cmd;
void cmd_callback(const std_msgs::String::ConstPtr& msg){
    cmd = msg->data;
    if (cmd == "0" || cmd == "emg" || cmd == "EMERGENCY"){
        EMERGENCY = true;
    }
    if (cmd == "233" || cmd == "ok"){
        EMERGENCY = false;
    }
}


template<typename T>
bool is_near(T a, double r){
    return MyDataFun::dis(a, current_pos_raw) <= r;
}

template<typename T>
bool is_near_2d(T a, double r){
    return MyDataFun::dis_2d(a, current_pos_raw) <= r;
}


bool takeoff_land(int task) {
    dji_osdk_ros::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service.call(droneTaskControl);

    if (!droneTaskControl.response.result) {
        ROS_ERROR("takeoff_land fail");
        return false;
    }
    ROS_INFO("Takeoff/Land Success!");

    return true;
}

bool obtain_control() {
    dji_osdk_ros::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    ROS_INFO("obtain control successful!");
    return true;
}

bool monitoredTakeoff() {
    ros::Time start_time = ros::Time::now();

    if (!takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1.1: Spin the motor
    while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
           display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
           ros::Time::now() - start_time < ros::Duration(5)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(5)) {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
    } else {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
    }

    // Step 1.2: Get in to the air
    while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
           (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(20)) {
        ROS_ERROR(
            "Takeoff failed. Aircraft is still on the ground, but the motors "
            "are "
            "spinning.");
        return false;
    } else {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
    }

    // Final check: Finished takeoff
    while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (display_mode != DJISDK::DisplayMode::MODE_P_GPS ||
        display_mode != DJISDK::DisplayMode::MODE_ATTITUDE) {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
    } else {
        ROS_ERROR(
            "Takeoff finished, but the aircraft is in an unexpected mode. "
            "Please "
            "connect DJI GO.");
        return false;
    }

    return true;
}

void M210_hold_ctrl(double _z = 0.0) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY |
         DJISDK::YAW_RATE | DJISDK::HORIZONTAL_BODY | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(_z);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_adjust_yaw(double _yaw) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY |
         DJISDK::YAW_ANGLE | DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(0);
    controlCMD.axes.push_back(_yaw);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_body_vel_yaw_rate_ctrl(double _x, double _y, double _z, double _yaw_rate) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY |
         DJISDK::YAW_RATE | DJISDK::HORIZONTAL_BODY | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(_x);
    controlCMD.axes.push_back(_y);
    controlCMD.axes.push_back(_z);
    controlCMD.axes.push_back(_yaw_rate);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_velocity_yaw_ctrl(double _vx, double _vy, double _vz, double _yaw) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY |
         DJISDK::YAW_ANGLE | DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(_vx);
    controlCMD.axes.push_back(_vy);
    controlCMD.axes.push_back(_vz);
    controlCMD.axes.push_back(_yaw);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_velocity_yaw_rate_ctrl(double _vx, double _vy, double _vz, double _yaw_rate) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY |
         DJISDK::YAW_RATE | DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(_vx);
    controlCMD.axes.push_back(_vy);
    controlCMD.axes.push_back(_vz);
    controlCMD.axes.push_back(_yaw_rate);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_position_yaw_ctrl(double _x, double _y, double _z, double _yaw) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_POSITION | DJISDK::HORIZONTAL_POSITION |
         DJISDK::YAW_ANGLE | DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(_x);
    controlCMD.axes.push_back(_y);
    controlCMD.axes.push_back(_z);
    controlCMD.axes.push_back(_yaw);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}

void M210_position_yaw_rate_ctrl(double _x, double _y, double _z, double _yaw) {
    sensor_msgs::Joy controlCMD;
    uint8_t flag =
        (DJISDK::VERTICAL_POSITION | DJISDK::HORIZONTAL_POSITION |
         DJISDK::YAW_RATE | DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE);
    controlCMD.axes.push_back(_x);
    controlCMD.axes.push_back(_y);
    controlCMD.axes.push_back(_z);
    controlCMD.axes.push_back(_yaw);
    controlCMD.axes.push_back(flag);

    ctrl_cmd_pub.publish(controlCMD);
}


template<typename T>
void UAV_velocity_yaw_rate_ctrl(T _pos_diff, double _yaw_diff){
    T vel;
    vel.x = _pos_diff.x * X_KP;
    vel.y = _pos_diff.y * Y_KP;
    vel.z = _pos_diff.z * Z_KP;
    double yaw_rate = _yaw_diff * YAW_KP;
    geometry_msgs::Vector3 sat;
    sat.x = 0.4;
    sat.y = 0.4;
    sat.z = 0.2;
    MyDataFun::saturate_vel(vel, sat);
    ROS_INFO("Velo cmd: %s", MyDataFun::output_str(vel).c_str());
    M210_velocity_yaw_rate_ctrl(vel.x, vel.y, vel.z, yaw_rate);
}

template<typename T>
void UAV_Control_to_Point_facing_it(T ctrl_cmd){
    double yaw_diff = MyDataFun::angle_2d(current_pos_raw, ctrl_cmd) - current_euler_angle.z;
    yaw_diff = MyMathFun::rad_round(yaw_diff);
    if (MyDataFun::dis_2d(ctrl_cmd, current_pos_raw) <= 1) yaw_diff = 0;
    ROS_INFO("Yaw diff: %.2lf", yaw_diff);
    UAV_velocity_yaw_rate_ctrl(MyDataFun::minus(ctrl_cmd, current_pos_raw), yaw_diff);
}

template<typename T>
void UAV_Control_to_Point_with_yaw(T ctrl_cmd, double _yaw){
    double yaw_diff = _yaw - current_euler_angle.z;
    yaw_diff = MyMathFun::rad_round(yaw_diff);
    if (MyDataFun::dis_2d(ctrl_cmd, current_pos_raw) <= 1) yaw_diff = 0;
    ROS_INFO("Yaw diff: %.2lf", yaw_diff);
    UAV_velocity_yaw_rate_ctrl(MyDataFun::minus(ctrl_cmd, current_pos_raw), yaw_diff);
}

template<typename T>
void UAV_Control_Body(T ctrl_cmd, double yaw_rate = 0.0){
    M210_body_vel_yaw_rate_ctrl(ctrl_cmd.x, ctrl_cmd.y, ctrl_cmd.z, yaw_rate);
}

void send_gimbal_angle_ctrl_cmd(double roll, double pitch, double yaw) {
    geometry_msgs::Vector3 v;
    v.x = roll;
    v.y = pitch;
    v.z = yaw;
    // ROS_INFO("rpy: %.2lf %.2lf %.2lf\n", roll, pitch, yaw);
    gimbal_angle_cmd_pub.publish(v);
}

void send_gimbal_speed_ctrl_cmd(double roll, double pitch, double yaw) {
    geometry_msgs::Vector3 v;
    v.x = roll;
    v.y = pitch;
    v.z = yaw;
    gimbal_speed_cmd_pub.publish(v);
}

double get_time_now(){
    return ros::Time::now().toSec();
}

bool enough_time_after(double _t0, double _duration){
    return get_time_now() -_t0 >= _duration;
}

bool set_local_position(){
  dji_osdk_ros::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
