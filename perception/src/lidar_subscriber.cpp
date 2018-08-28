#include "ros/ros.h"
#include "std_msgs/String.h"

#include <run_msg/lidar_msg.h>

run_msg::lidar_msg msg;

void lidarDataCallback(const run_msg::lidar_msg::ConstPtr& recv_msg) {
	msg = *recv_msg;

	ros::Time cur = ros::Time().now();
	int delay = (cur.sec - msg.header.stamp.sec) * 1e9;
	delay += cur.nsec - msg.header.stamp.nsec;
	delay /= 1e6;
	ROS_INFO("Recv lidar msg: [%d], delay: [%d]ms", recv_msg -> header.seq, delay);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "lidar_subscriber");

	ros::NodeHandle n;
	//Buffer scale: 1000
	ros::Subscriber lidar_sub = n.subscribe("lidarData", 1000, lidarDataCallback);
	
	ros::spin();
/*
	//recv frequency: 1 Hz
	ros::Rate loop_rate(1);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}
*/

	return 0;
}

