/**
 * 
 */
#include <pthread.h>
#include <ros/ros.h>
#include <dji_sdk/dji_sdk_node.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <opencv_apps/Flow.h>

ros::Time last_inst;
ros::Subscriber flows;
int xx, yy;
const double EPS = 1e-2;
void DJISDKNode::flowsCallback(const opencv_apps::FlowArrayStamped::ConstPtr& msg) {
	int size = msg->flow.size();
	xx = yy = 0;
	for (int i = 0; i < size; i++) {
		if (msg->flow[i].velocity.x > EPS) {
			xx++;
		} else if (msg->flow[i].velocity.x < -EPS) {
			xx--;
		}
		if (msg->flow[i].velocity.y > EPS) {
			yy++;
		} else {
			yy--;
		}
	}
}

void *run(void * arg) {
	pthread_detach(pthread_self());
	DJISDKNode* node = (DJISDKNode*)arg;
	while(1) {
		ROS_INFO("do something");
		if (ros::Time::now() - last_inst > ros::Duration(2)) {
			ROS_INFO("hover %d, %d", xx, yy);

		}
		ros::Duration(2).sleep();
	}
}
void DJISDKNode::hover(ros::NodeHandle& nh) {
	last_inst = ros::Time::now();
	flows = nh.subscribe<opencv_apps::FlowArrayStamped>("/fback_flow/flows", 10, &DJISDKNode::flowsCallback, this);
	pthread_t th;
	pthread_create(&th, NULL, run, this);
}

