#include "test_dji/NewUtils.h"
#include "test_dji/NewMyDataFun.h"
#include "test_dji/NewMyMathFun.h"
#include "test_dji/NewFlightControl.hpp"
#define CAMERA_ANGLE 30
// #define SEARCH_BODY
// #define ON_GROUND_TEST

using namespace dji_osdk_ros;
using namespace std;

typedef enum { TAKEOFF, ASCEND, SEARCH, PURSUE, HOLD, BACK, LAND } ControlState;
ControlState task_state;
#ifdef SEARCH_BODY
vector<pair<geometry_msgs::Vector3, double> > search_body;
size_t search_body_cnt;
double search_time;
#else
vector<geometry_msgs::Vector3> search_tra;
size_t search_tra_cnt;
#endif
double hold_begin_time, ascend_begin_time;

double pos_err_v[3];
Point vis_vsl_pos;
double vis_vsl_pix[2];
double q_LOS_v[3], euler_angle[3];

MyMathFun::XYZ_Median_Filter<Point> vsl_pos_fil;
int vls_pos_cnt;

template<typename T>
bool pos_valid(T a){
    return a.x >= -8.0 && a.x <= 18.0 && a.y >= -7.0 && a.y <= 7.0;
}

void det_callback(const std_msgs::Float32MultiArray::ConstPtr & msg){
    vis_vsl_pix[0] = msg->data[0];
    vis_vsl_pix[1] = msg->data[1];
    bool vis_vsl_flag = (msg->data[4] > 0.5);
    //ROS_INFO("Det Flag: %d", (vis_vsl_flag == true));
    if (!vis_vsl_flag) return;
    MyDataFun::set_value(euler_angle, current_gimbal_angle);
    euler_angle[0] = euler_angle[0] * DEG2RAD_COE;
    euler_angle[1] = -euler_angle[1] * DEG2RAD_COE;
    euler_angle[2] = MyMathFun::rad_round(PI / 2 - euler_angle[2] * DEG2RAD_COE);
    MyMathFun::angle_transf(euler_angle, 0.0, vis_vsl_pix, q_LOS_v);
    // printf("qlos: (%.2lf, %.2lf)\n", q_LOS_v[1] * RAD2DEG, q_LOS_v[2] * RAD2DEG);
    pos_err_v[2] = -current_pos_raw.z;	
    pos_err_v[0] = (-pos_err_v[2]/tan(q_LOS_v[1]))*cos(q_LOS_v[2]);
    pos_err_v[1] = (-pos_err_v[2]/tan(q_LOS_v[1]))*sin(q_LOS_v[2]);
    //ROS_INFO("Rel Pos of Target: (%.2lf, %.2lf, %.2lf)", pos_err_v[0], pos_err_v[1], pos_err_v[2]);
    vis_vsl_pos.x = current_pos_raw.x + pos_err_v[0];
    vis_vsl_pos.y = current_pos_raw.y + pos_err_v[1];
    vis_vsl_pos.z = current_pos_raw.z + pos_err_v[2];
    //ROS_INFO("Abs pos of target: %s\n", MyDataFun::output_str(vis_vsl_pos).c_str());
    //if (!pos_valid(vis_vsl_pos)) return;
    vsl_pos_fil.new_data(vis_vsl_pos);
    vis_vsl_pos = vsl_pos_fil.result();
    vls_pos_cnt++;
}

void toStepTakeoff(){
    task_state = TAKEOFF;
}

void toStepAscend(){
    ascend_begin_time = get_time_now();
    task_state = ASCEND;
}

void toStepSearch(){
#ifdef SEARCH_BODY
    search_body_cnt = 0;
    search_time = get_time_now();
#else
    search_tra_cnt = 0;
#endif
    vls_pos_cnt = 0;
    task_state = SEARCH;
}

void toStepPursue(){
    task_state = PURSUE;
}

void toStepHold(){
    task_state = HOLD;
    hold_begin_time = get_time_now();
}

void toStepLand(){
    task_state = LAND;
}

void StepTakeoff() {
    ROS_INFO("###----StepTakeoff----###");
    double expected_height = 1.4;
    ROS_INFO("Expected height @ %.2lf", expected_height);
    M210_position_yaw_rate_ctrl(0, 0, expected_height, 0);
    if (MyMathFun::nearly_is(current_pos_raw.z, expected_height, 0.2)){
        // ROS_INFO("Arrive expected height @ %.2lf", expected_height);
        // toStepSearch();
        // toStepAscend();
        toStepHold();
    }
}

void StepAscend(){
    ROS_INFO("###----StepAscend----###");
    double ascend_time = 3.0;
    ROS_INFO("Ascending by 0.1m/s: %.2lf", get_time_now() - ascend_begin_time);
    M210_hold_ctrl(0.1);
    if (enough_time_after(ascend_begin_time, ascend_time)){
        toStepHold();
    }
}

void StepSearch() {
#ifdef SEARCH_BODY
    ROS_INFO("###----StepSearch(Body)----###");
    double tol = 0.5;
    geometry_msgs::Vector3 desired_velo;
    MyDataFun::set_value(desired_velo, search_body[search_body_cnt].first);
    ROS_INFO("Go by %s(%ld th) Time: %.2lf", MyDataFun::output_str(desired_velo).c_str(), search_body_cnt, get_time_now() - search_time);
    ROS_INFO("Target cnt: %d", vls_pos_cnt);
    UAV_Control_Body(desired_velo);
    if (enough_time_after(search_time, search_body[search_body_cnt].second)){
        search_body_cnt++;
        search_time = get_time_now();
        if (search_body_cnt == search_body.size()){
            toStepHold();
        }
    }
    
    if (vls_pos_cnt >= 100){
        toStepPursue();
    }
#else
    ROS_INFO("###----StepSearch(Ground)----###");
    double tol = 0.2;
    geometry_msgs::Vector3 desired_point;
    MyDataFun::set_value(desired_point, search_tra[search_tra_cnt]);
    ROS_INFO("Go to %s(%ld th)", MyDataFun::output_str(desired_point).c_str(), search_tra_cnt);
    ROS_INFO("Target cnt: %d", vls_pos_cnt);
    // UAV_Control_to_Point_facing_it(desired_point);
    UAV_Control_to_Point_with_yaw(desired_point, yaw_offset);
    if (is_near(desired_point, tol)){
        search_tra_cnt++;
        if (search_tra_cnt == search_tra.size()){
            toStepHold();
        }
    }
    
    if (vls_pos_cnt >= 100){
        toStepPursue();
    }
#endif
}

void StepPursue(){
    ROS_INFO("###----StepPursue----###");
    double tol = 1.5;
    geometry_msgs::Vector3 desired_point;
    MyDataFun::set_value(desired_point, vis_vsl_pos);
    // desired_point.x = 3;
    // desired_point.y = 0;
    desired_point.z = 1.0;
    ROS_INFO("Pursue to %s", MyDataFun::output_str(desired_point).c_str());
    UAV_Control_to_Point_facing_it(desired_point);
    if (is_near(desired_point, tol)){
        toStepHold();
    }
}

void StepHold() {
    ROS_INFO("###----StepHold----###");
    double hold_time = 6.0;
    ROS_INFO("Hold %.2lf", get_time_now() - hold_begin_time);
    // M210_hold_ctrl(0.0);
    M210_adjust_yaw(yaw_offset);
    if (enough_time_after(hold_begin_time, hold_time)){
        toStepLand();
    }
}

void StepBack() {
    ROS_INFO("###----StepBack----###");
}

void StepLand() {
    ROS_INFO("###----StepLand----###");
    ROS_INFO("Landing...");
    takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_LAND);
    // task_state = BACK;
}

void ControlStateMachine() {
    switch (task_state) {
        case TAKEOFF: {
            StepTakeoff();
            break;
        }
        case SEARCH: {
            StepSearch();
            break;
        }
        case PURSUE: {
            StepPursue();
            break;
        }
        case HOLD: {
            StepHold();
            break;
        }
        case BACK: {
            StepBack();
            break;
        }
        case LAND: {
            StepLand();
            break;
        }
        default: {
            StepHold();
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbzirc_demo_search", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    
	string uav_name = "none";
	if (argc > 1) {
		uav_name = std::string(argv[1]);
	}
	string det_topic = "/no_det";
    if (argc > 2 && std::string(argv[2]) == "on") {
	    det_topic = "/detect_results";
	}
    while (uav_name == "none"){
        ROS_ERROR("Invalid vehicle name: %s", uav_name.c_str());
    }
	ROS_INFO("Vehicle name: %s", uav_name.c_str());

    ros::Subscriber attitudeSub =
        nh.subscribe(uav_name + "/dji_osdk_ros/attitude", 10, &attitude_callback);
    ros::Subscriber gimbal_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/gimbal_angle", 10, &gimbal_callback);
    ros::Subscriber height_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/height_above_takeoff", 10, &height_callback);
    ros::Subscriber vo_pos_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/vo_position", 10, &vo_pos_callback);
    ros::Subscriber range_pos_sub = 
        nh.subscribe(uav_name + "/filter/odom", 10, &range_pos_callback);
	ros::Subscriber flightStatusSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/flight_status", 10, &flight_status_callback);
	ros::Subscriber displayModeSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/display_mode", 10, &display_mode_callback);
    ros::Subscriber cmd_sub = 
            nh.subscribe(uav_name + "/commander_cmd", 10, &cmd_callback);
    gimbal_angle_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_angle_cmd", 10);
    gimbal_speed_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_speed_cmd", 10);
    ros::Subscriber local_pos_sub = nh.subscribe(uav_name + "/dji_osdk_ros/local_position", 10, &local_pos_callback);

    ctrl_cmd_pub = nh.advertise<sensor_msgs::Joy>(
        uav_name + "/dji_osdk_ros/flight_control_setpoint_generic", 10);
    sdk_ctrl_authority_service =
        nh.serviceClient<dji_osdk_ros::SDKControlAuthority>(
            uav_name + "/dji_osdk_ros/sdk_control_authority");
    drone_task_service = nh.serviceClient<dji_osdk_ros::DroneTaskControl>(
        uav_name + "/dji_osdk_ros/drone_task_control");
    set_local_pos_reference    = nh.serviceClient<dji_osdk_ros::SetLocalPosRef> (uav_name + "/dji_osdk_ros/set_local_pos_ref");

    ros::Rate rate(50);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }


#ifdef SEARCH_BODY
    search_body.push_back(make_pair(MyDataFun::new_point(0.1, 0, 0), 10));
    search_body.push_back(make_pair(MyDataFun::new_point(0, 0.1, 0), 10));
    search_body.push_back(make_pair(MyDataFun::new_point(-0.1, 0, 0), 10));
    search_body.push_back(make_pair(MyDataFun::new_point(0, -0.1, 0), 10));


#else
    yaw_offset = current_euler_angle.z;
    ROS_INFO("Yaw offset: %.2lf", yaw_offset * RAD2DEG_COE);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }
    MyDataFun::set_value(position_offset, current_pos_raw);
    ROS_INFO("Position offset: %s", MyDataFun::output_str(position_offset).c_str());
    //search_tra.push_back(MyDataFun::new_point(3.0, 0.0, 1.4));
    //search_tra.push_back(MyDataFun::new_point(3.0, -2.0, 1.4));
    //search_tra.push_back(MyDataFun::new_point(0.0, -2.0, 1.4));
    //search_tra.push_back(MyDataFun::new_point(0.0, 0.0, 1.4));
   
    double y_dir;
    if (uav_name == "suav_1") y_dir = -3.0;
    else if (uav_name == "suav_2") y_dir = 3.0; 
    search_tra.push_back(compensate_offset(MyDataFun::new_point(10.0, 0.0, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(10.0, y_dir, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0.0, y_dir, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0.0, 0.0, 2.0)));

	ROS_INFO("Subscribe to topic: %s%s", uav_name.c_str(), det_topic.c_str());

    ROS_INFO("Search Trajectory:");
    for (auto a: search_tra){
        ROS_INFO("%s", MyDataFun::output_str(a).c_str());
    }
#ifdef GPS_HEIGHT
    ROS_INFO("Use GPS for height");
#else
    ROS_INFO("Use supersonic wave for height");
#endif
    string confirm_input;
    while (confirm_input != "yes"){
        ROS_INFO("Confirm: yes/no");
        cin >> confirm_input;
        if (confirm_input == "no"){
            return 0;
        }
    }

#endif

    ROS_INFO("Reset Gimbal");
    send_gimbal_angle_ctrl_cmd(0, 0, 0);
    sleep(2);

    ROS_INFO("Set Gimbal");
    send_gimbal_angle_ctrl_cmd(0, -30, 0);
    sleep(2);

    
    // if (!set_local_position()){
    //     ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    //     return 1;
    // }
    

    ROS_INFO("Waiting for command to take off...");
    sleep(3);
    // while(cmd != "ok"){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    #ifndef ON_GROUND_TEST
    obtain_control();
    monitoredTakeoff();
    #endif
    ros::Subscriber det_sub = 
        nh.subscribe(uav_name + det_topic, 10, &det_callback);


    ROS_INFO("Start Control State Machine...");
    toStepTakeoff();

    while (ros::ok()) {
        // std::cout << "\033c" << std::flush;
        ROS_INFO("-----------");
        ROS_INFO("M210(State: %d) @ %s", task_state, MyDataFun::output_str(current_pos_raw).c_str());
        // ROS_INFO("Gimbal %s", MyDataFun::output_str(current_gimbal_angle).c_str());
        ROS_INFO("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", current_euler_angle.x * RAD2DEG_COE,
                                                    current_euler_angle.y * RAD2DEG_COE,
                                                    current_euler_angle.z * RAD2DEG_COE);
        // ROS_INFO("Pixel error: %.2lf, %.2lf", vis_vsl_pix[0], vis_vsl_pix[1]);
        // ROS_INFO("qLos: %.2lf, %.2lf, %.2lf", q_LOS_v[0], q_LOS_v[1], q_LOS_v[2]);
        // ROS_INFO("Gimbal euler: %.2lf, %.2lf, %.2lf\n", euler_angle[0], euler_angle[1], euler_angle[2]);
        ROS_INFO("Target relative pos: %s", MyDataFun::output_str(MyDataFun::new_point(pos_err_v)).c_str());
        ROS_INFO("Target absolute pos: %s", MyDataFun::output_str(vis_vsl_pos).c_str());
        ROS_INFO("Detection cnt: %d, Filter cnt: %ld", vls_pos_cnt, vsl_pos_fil.x.v.size());
    #ifndef ON_GROUND_TEST
        if (EMERGENCY){
            M210_hold_ctrl();
            printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
        }
        else {
            ControlStateMachine();
        }
    #endif
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
