#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//#include <run_msg/lidar_msg.h>
#include <visualization_msgs/Marker.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ> pc2;
pcl::PointCloud<pcl::PointXYZRGB> pc2_rgb;

ros::Publisher lidar_pub;
ros::Publisher marker_pub;

void algorithm() {
	ROS_INFO("Run algorithm...");
	pc2_rgb.clear();
	pc2_rgb.height = 1;
	pc2_rgb.width = pc2.width;
	pc2_rgb.resize(pc2.points.size());
	for(int i = 0; i < pc2.points.size(); i++) {
		pc2_rgb.points[i].x = pc2.points[i].x;
		pc2_rgb.points[i].y = pc2.points[i].y;
		pc2_rgb.points[i].z = 0;

		pc2_rgb.points[i].r = 255;
		pc2_rgb.points[i].g = 0;
		pc2_rgb.points[i].b = 0;
	}

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(pc2_rgb, output);
	output.header.frame_id = "pandar";
	lidar_pub.publish(output);
}

void drawLine() {
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "pandar";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 1;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.004;

	line_list.color.g = 1.0;
	line_list.color.a = 1.0;

	geometry_msgs::Point p;
	
	int grid_size = 100;
	double grid_resolution = 10;

	for(int i = -1*grid_size; i < grid_size; i++) {
		p.x = i / grid_resolution;
		p.y = -1*grid_size / grid_resolution;
		p.z = 0;
		line_list.points.push_back(p);
		p.y = grid_size / grid_resolution;
		line_list.points.push_back(p);

		p.x = -1*grid_size / grid_resolution;
		p.y = i / grid_resolution;
		p.z = 0;
		line_list.points.push_back(p);
		p.x = grid_size / grid_resolution;
		line_list.points.push_back(p);
	}
	ROS_INFO("line list size = [%d]", line_list.points.size());
	marker_pub.publish(line_list);
}

void lidarDataCallback(const sensor_msgs::PointCloud2ConstPtr& recv_msg) {
	pcl::PCLPointCloud2 cloud;

	pcl_conversions::toPCL(*recv_msg, cloud);
	pcl::fromPCLPointCloud2(cloud, pc2);
	ROS_INFO("Recv point cloud data, size = [%d]", pc2.points.size());

	if(pc2.points.size() > 0) {
		algorithm();
		drawLine();
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "lidar_subscriber");

	ros::NodeHandle n;
	ros::Subscriber lidar_sub = n.subscribe("pandar_points", 10, lidarDataCallback);
	lidar_pub = n.advertise<sensor_msgs::PointCloud2> ("pandar_process", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);

	ros::spin();

	return 0;
}

