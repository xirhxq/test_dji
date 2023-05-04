#include<ros/ros.h>
#include<dji_osdk_ros/VOPosition.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "test_dji", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<dji_osdk_ros::VOPosition>("test_dji", 1000);
	ros::Rate loop_rate(10);
	while (ros::ok()){
		dji_osdk_ros::VOPosition msg;
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
