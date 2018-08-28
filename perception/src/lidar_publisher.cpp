#include "ros/ros.h"
#include "std_msgs/String.h"

#include <run_msg/lidar_msg.h>
#include <sstream>


int main(int argc, char **argv) {

	ros::init(argc, argv, "lidar_publisher");

	ros::NodeHandle n;
	//Buffer scale: 1000. If the queue reacher 1000 msgs, it will throw away old msgs
	ros::Publisher lidar_pub = n.advertise<run_msg::lidar_msg>("lidarData", 1000);

	//send frequency: 10 Hz
	ros::Rate loop_rate(10);
	int count = 0;

	while(ros::ok()) {
		run_msg::lidar_msg msg;
		msg.header.seq = count++;
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = "";
		
		lidar_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

