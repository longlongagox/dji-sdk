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
pthread_mutex_t mutex;
int xx, yy; //
const double EPS = 1.0;
const float velocity = 1.0;
const int LIMIT = 50;

void DJISDKNode::flowsCallback(const opencv_apps::FlowArrayStamped::ConstPtr& msg) {
	int size = msg->flow.size();
	if (pthread_mutex_lock(&mutex) != 0) {
		ROS_ERROR("lock error!");
		return ;
	}
	xx = yy = 0;
	for (int i = 0; i < size; i++) {
		if (msg->flow[i].velocity.x > EPS) {
			xx++;
		} else if (msg->flow[i].velocity.x < -EPS) {
			xx--;
		}
		if (msg->flow[i].velocity.y > EPS) {
			yy++;
		} else if (msg->flow[i].velocity.y < -EPS) {
			yy--;
		}
	}
	pthread_mutex_unlock(&mutex);
}

void *run(void * arg) {
	pthread_detach(pthread_self());
	DJISDKNode* node = (DJISDKNode*)arg;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	while(1) {
		if (ros::Time::now() - last_inst > ros::Duration(2)) {
			try{
				if (pthread_mutex_lock(&mutex) != 0) {
					ROS_ERROR("lock error!");
					return (void*)NULL;
				}
				ROS_INFO("hover %d, %d", xx, yy);
		    	listener.lookupTransform("/usb_cam", "/body_FLU", ros::Time(0), transform);
		    	tf::Vector3 v_cam(0, 0, 0);
		    	if (xx > LIMIT) {
		    		v_cam.setX(-1.0);
		    	} else if (xx < -LIMIT) {
		    		v_cam.setX(1.0);
		    	}
		    	if (yy > LIMIT) {
		    		v_cam.setY(-1.0);
		    	} else if (yy < -LIMIT) {
		    		v_cam.setY(1.0);
		    	}
		    	pthread_mutex_unlock(&mutex);

		    	ROS_INFO("transform.getBasis()");
		    	ROS_INFO("%f\t%f\t%f", transform.getBasis()[0].getX(), transform.getBasis()[0].getY(), transform.getBasis()[0].getZ());
		    	ROS_INFO("%f\t%f\t%f", transform.getBasis()[1].getX(), transform.getBasis()[1].getY(), transform.getBasis()[1].getZ());
		    	ROS_INFO("%f\t%f\t%f", transform.getBasis()[2].getX(), transform.getBasis()[2].getY(), transform.getBasis()[2].getZ());
		    	ROS_INFO("----------------------------------");
		    	ROS_INFO("transform.getRotation()");
		    	ROS_INFO("%f, %f, %f, %f", transform.getRotation().x(), transform.getRotation().y(),
		    			 transform.getRotation().z(), transform.getRotation().w());
		    	ROS_INFO("---------------");

		    	v_cam.setZ(0);
		    	tf::Vector3 rotation;
		    	rotation.setX(v_cam.getX()*transform.getBasis()[0].getX()+v_cam.getY()*transform.getBasis()[1].getX()
		    			+v_cam.getZ()*transform.getBasis()[2].getX());

		    	rotation.setY(v_cam.getX()*transform.getBasis()[0].getY()+v_cam.getY()*transform.getBasis()[1].getY()
		    			+v_cam.getZ()*transform.getBasis()[2].getY());

		    	rotation.setZ(v_cam.getX()*transform.getBasis()[0].getZ()+v_cam.getY()*transform.getBasis()[1].getZ()
		    			+v_cam.getZ()*transform.getBasis()[2].getZ());
		    	ROS_INFO("velocity_cam: %f, %f, %f", v_cam.x(), v_cam.y(), v_cam.z());
		    	ROS_INFO("velocity_rotation: %f, %f, %f", rotation.x(), rotation.y(), rotation.z());

		    	uint8_t flag = (Control::VERTICAL_VELOCITY |
				                  Control::HORIZONTAL_VELOCITY |
				                  Control::YAW_RATE |
				                  Control::HORIZONTAL_GROUND |
				                  Control::STABLE_ENABLE);
				float vx        = rotation.x()*velocity;
				float vy        = rotation.y()*velocity;
				float vz        = rotation.z()*velocity;
				float yawRate   = 0.0;

			    node->flightControl(flag, vx, vy, vz, yawRate);

		    } catch (tf::TransformException &ex) {
		    	ROS_ERROR("%s",ex.what());
		    }

		}
		ros::Duration(2).sleep();
	}
}
void DJISDKNode::hover(ros::NodeHandle& nh) {
	last_inst = ros::Time::now();
	if (pthread_mutex_init(&mutex, NULL) != 0) {
		ROS_ERROR("Init mutex error!!");
		exit(1);
	}
	flows = nh.subscribe<opencv_apps::FlowArrayStamped>("/simple_flow/flows", 10, &DJISDKNode::flowsCallback, this);
	pthread_t th;
	pthread_create(&th, NULL, run, this);
}

