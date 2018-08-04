#include <ymg_gnss_filter/gnss_filter.hpp>
#include <Eigen/Geometry>

GnssFilter::GnssFilter()
	: tf2_listener_( tf2_buffer_ )
{/*{{{*/
	ros::NodeHandle nh, private_nh("~");
	private_nh.param("odom_frame", odom_frame_, odom_frame_ );
	private_nh.param("odom_gnss_frame", robot_gnss_frame_, robot_gnss_frame_ );
	private_nh.param("robot_frame", robot_frame_, robot_frame_ );
	private_nh.param("min_store_n", min_store_n_, min_store_n_ );
	private_nh.param("store_sec", store_sec_, store_sec_ );
	private_nh.param("sd_threshold_ratio", sd_threshold_ratio_, sd_threshold_ratio_ );

	robot_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>( "gnss_robot_pose", 1 );
	pose_cloud_publisher_ = nh.advertise<geometry_msgs::PoseArray>( "gnss_robot_cloud", 1 );
	// gnss_fix_subscriber_ = nh.subscribe( "/fix", 100, &GnssFilter::doFiltering, this );
	gnss_twist_subscriber_ = nh.subscribe( "/vel", 100, &GnssFilter::twistCallback, this );
	gnss_point_subscriber_ = nh.subscribe( "/gnss_point", 100, &GnssFilter::pointCallback, this );
}/*}}}*/

void GnssFilter::spin()
{/*{{{*/
	ros::spin();
}/*}}}*/

void GnssFilter::twistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
{/*{{{*/
	ROS_INFO( "twist callback" );
	double vel_x = msg->twist.linear.x;
	double vel_y = msg->twist.linear.y;
	robot_pose2d_.theta = atan2( vel_y, vel_x );
}/*}}}*/

void GnssFilter::pointCallback( const geometry_msgs::PointStamped::ConstPtr& msg )
{/*{{{*/
	ROS_INFO( "point callback" );
	header_ = msg->header;
	robot_pose2d_.x = msg->point.x;
	robot_pose2d_.y = msg->point.y;
	publishPose();
	broadcastTF();
}/*}}}*/

void GnssFilter::doFiltering( const geometry_msgs::PoseStamped::ConstPtr& msg )
{/*{{{*/
	ROS_INFO( "do filtering" );
	removeOldPose( msg->header.stamp );
	motionUpdate( msg->header.stamp );
	updatePoselist( *msg );
	publishParticleCloud( msg->header );
}/*}}}*/

void GnssFilter::removeOldPose( const ros::Time stamp )
{/*{{{*/
	std::vector<Pose2dWithTime> new_poselist;
	for ( const auto& pose : poselist_ ) {
		if ( stamp - pose.timestamp < ros::Duration( store_sec_ ) ) {
			new_poselist.push_back( pose );
		}
	}
	poselist_ = new_poselist;
}/*}}}*/

void GnssFilter::motionUpdate( const ros::Time stamp )
{/*{{{*/
	static geometry_msgs::TransformStamped tf;
	static geometry_msgs::TransformStamped tf_past;
	static bool is_first_time = true;

	// get odom -> robot tf
	try {
		tf = tf2_buffer_.lookupTransform( odom_frame_, robot_frame_, stamp, ros::Duration( 0.1 ) );
	}
	catch ( tf2::TransformException &ex ) {
		ROS_INFO( "[GnssFilter] cannot lookup tf ( from %s to %s )", odom_frame_.c_str(), robot_frame_.c_str() );
		ros::Duration(1.0).sleep();
		return;
	}

	if ( is_first_time ) {
		is_first_time = false;
		return;
	}

	double diff_x = tf.transform.translation.x - tf_past.transform.translation.x;
	double diff_y = tf.transform.translation.y - tf_past.transform.translation.y;
	double r = sqrt( diff_x * diff_x + diff_y * diff_y );
	double theta_gl = atan2( diff_y, diff_x );
	double theta_p1 = tf::getYaw( tf_past.transform.rotation );
	double theta_p2 = tf::getYaw( tf.transform.rotation );
	double theta = theta_gl - theta_p1;
	double motion_theta = theta_p2 - theta_p1;

	for ( auto& pose : poselist_ ) {
		double motion_x = r * cos( pose.pose2d.theta + theta );
		double motion_y = r * sin( pose.pose2d.theta + theta );
		pose.pose2d.x += motion_x;
		pose.pose2d.y += motion_y;
		pose.pose2d.theta += motion_theta;
	}

	tf_past = tf;
}/*}}}*/

void GnssFilter::updatePoselist( const geometry_msgs::PoseStamped& measured_pose )
{/*{{{*/
	if ( poselist_.size() < min_store_n_ ) {
		Pose2dWithTime new_pose;
		new_pose.pose2d.x = measured_pose.pose.position.x;
		new_pose.pose2d.y = measured_pose.pose.position.y;
		new_pose.pose2d.theta = tf::getYaw( measured_pose.pose.orientation );
		new_pose.timestamp = measured_pose.header.stamp;
		poselist_.push_back( new_pose );
		return;
	}

	// calc mean
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	for ( auto& pose : poselist_ ) {
		mean.x() += pose.pose2d.x;
		mean.y() += pose.pose2d.y;
	}
	mean /= poselist_.size();
	gnss_mean = mean;

	// calc standard deviation
	double standard_deviation = 0.0;
	for ( auto& pose : poselist_ ) {
		double diff_x = mean.x() - pose.pose2d.x;
		double diff_y = mean.y() - pose.pose2d.y;
		standard_deviation += diff_x * diff_x + diff_y * diff_y;
	}
	standard_deviation = sqrt( standard_deviation / poselist_.size() );
	gnss_sd = standard_deviation;

	// calc measured_pose error
	Eigen::Vector2d measured_p;
	measured_p.x() = measured_pose.pose.position.x;
	measured_p.y() = measured_pose.pose.position.y;
	double error = ( mean - measured_p ).norm();

	// add new pose if error is min
	if ( error < gnss_sd * sd_threshold_ratio_ ) {
		Pose2dWithTime new_pose;
		new_pose.timestamp = measured_pose.header.stamp;
		new_pose.pose2d.x = measured_pose.pose.position.x;
		new_pose.pose2d.y = measured_pose.pose.position.y;
		new_pose.pose2d.theta = tf::getYaw( measured_pose.pose.orientation );
		poselist_.push_back( new_pose );
	}
}/*}}}*/

void GnssFilter::publishParticleCloud( const std_msgs::Header header )
{/*{{{*/
	static int seq = 0;
	geometry_msgs::PoseArray pose_array_msg;
	pose_array_msg.header = header;

	for ( const auto& pose : poselist_ ) {
		geometry_msgs::Pose a_pose;
		a_pose.position.x = pose.pose2d.x;
		a_pose.position.y = pose.pose2d.y;
		a_pose.position.z = 0.0;
		geometry_msgs::Quaternion q;
		quaternionTFToMsg( tf::createQuaternionFromRPY(0.0, 0.0, pose.pose2d.theta), q );
		a_pose.orientation = q;
		pose_array_msg.poses.push_back( a_pose );
	}

	pose_cloud_publisher_.publish( pose_array_msg );
}/*}}}*/

void GnssFilter::publishPose()
{/*{{{*/
	geometry_msgs::PoseStamped msg;
	msg.header = header_; 
	msg.pose.position.x = robot_pose2d_.x;
	msg.pose.position.y = robot_pose2d_.y;
	msg.pose.position.z = 0.0;
	geometry_msgs::Quaternion q;
	quaternionTFToMsg( tf::createQuaternionFromRPY(0.0, 0.0, robot_pose2d_.theta), q );
	msg.pose.orientation = q;
	robot_pose_publisher_.publish( msg );
}/*}}}*/

void GnssFilter::broadcastTF()
{/*{{{*/
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3( robot_pose2d_.x, robot_pose2d_.y, 0.0 ) );
	tf::Quaternion q;
	q.setRPY( 0.0, 0.0, robot_pose2d_.theta );
	transform.setRotation(q);
	br.sendTransform( tf::StampedTransform( transform, header_.stamp, gnss_frame_, robot_gnss_frame_ ) );
}/*}}}*/

