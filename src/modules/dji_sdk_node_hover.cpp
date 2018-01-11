/**
 * 
 */
#include <pthread.h>
#include <ros/ros.h>
#include <dji_sdk/dji_sdk_node.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Joy.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <opencv_apps/Flow.h>


ros::Time last_inst;
ros::Subscriber flows;
int xx, yy;
const double EPS = 1e-2;
const double velocity = 1.0;
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
	tf::TransformListener listener;
	tf::StampedTransform transform;
	while(1) {
		if (ros::Time::now() - last_inst > ros::Duration(2)) {
			ROS_INFO("hover %d, %d", xx, yy);
			try{
		    	listener.lookupTransform("/usb_cam", "/ground_ENU", ros::Time(0), transform);
		    	tf::Vector3 v_cam;
		    	if (xx > 0) {
		    		v_cam.setX(-1.0);
		    	} else {
		    		v_cam.setX(1.0);
		    	}
		    	if (yy > 0) {
		    		v_cam.setY(-1.0);
		    	} else {
		    		v_cam.setY(1.0);
		    	}

		    	v_cam.setZ(0);
		    	tf::Vector3 rotation;
		    	// rotation.setX(v_cam.getX()*transform.getBasis()[0].getX()+v_cam.getY()*transform.getBasis()[1].getX()
		    	// 		+v_cam.getZ()*transform.getBasis()[2].getX());

		    	// rotation.setY(v_cam.getX()*transform.getBasis()[0].getY()+v_cam.getY()*transform.getBasis()[1].getY()
		    	// 		+v_cam.getZ()*transform.getBasis()[2].getY());

		    	// rotation.setZ(v_cam.getX()*transform.getBasis()[0].getZ()+v_cam.getY()*transform.getBasis()[1].getZ()
		    	// 		+v_cam.getZ()*transform.getBasis()[2].getZ());

		    } catch (tf::TransformException &ex) {
		    	ROS_ERROR("%s",ex.what());
		    }

		}
		ros::Duration(2).sleep();
	}
}
void DJISDKNode::hover(ros::NodeHandle& nh) {
	last_inst = ros::Time::now();
	ROS_INFO("set last %s", __func__);
	flows = nh.subscribe<opencv_apps::FlowArrayStamped>("/fback_flow/flows", 10, &DJISDKNode::flowsCallback, this);
	pthread_t th;
	pthread_create(&th, NULL, run, this);
}

