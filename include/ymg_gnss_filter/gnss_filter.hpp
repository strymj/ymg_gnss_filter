#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GnssFilter {
	public:
		GnssFilter();
		void spin();
	private:
		// void doFiltering();
		void updatePoselist();
		std::string odom_frame_ = "odom";
		std::string robot_frame_ = "base_link";
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
		ros::Publisher robot_pose_publisher_;
		ros::Subscriber gnss_subscriber_;
		std::vector<geometry_msgs::Pose> poselist_;
}; // class GnssFilter
