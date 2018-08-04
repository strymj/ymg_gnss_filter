#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

class GnssFilter {
	public:
		GnssFilter();
		void spin();
		void twistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg );
		void pointCallback( const geometry_msgs::PointStamped::ConstPtr& msg );
		void doFiltering( const geometry_msgs::PoseStamped::ConstPtr& msg );
	private:
		void removeOldPose( const ros::Time stamp );
		void motionUpdate( const ros::Time stamp );
		void updatePoselist( const geometry_msgs::PoseStamped& msg );
		void publishParticleCloud( const std_msgs::Header header );
		void publishPose();
		void broadcastTF();
		std_msgs::Header header_;
		std::string odom_frame_ = "odom";
		std::string gnss_frame_ = "gnss";
		std::string robot_frame_ = "base_link_yp";
		std::string robot_gnss_frame_ = "base_link";

		Eigen::Vector2d gnss_mean;
		double gnss_sd;   // dtandard deviation
		int min_store_n_ = 20;
		double store_sec_ = 10.0;
		double sd_threshold_ratio_ = 1.0;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
		ros::Publisher robot_pose_publisher_, pose_cloud_publisher_;
    ros::Subscriber gnss_fix_subscriber_, gnss_twist_subscriber_, gnss_point_subscriber_;
		geometry_msgs::Pose2D robot_pose2d_;
		struct Pose2dWithTime {
			ros::Time timestamp;
			geometry_msgs::Pose2D pose2d;
		};
		std::vector<Pose2dWithTime> poselist_;
}; // class GnssFilter
