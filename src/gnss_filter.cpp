#include <ymg_gnss_filter/gnss_filter.hpp>

GnssFilter::GnssFilter()
	: tf2_listener_( tf2_buffer_ )
{/*{{{*/
	ros::NodeHandle nh, private_nh("~");
	private_nh.param("odom_frame", odom_frame_, odom_frame_ );
	private_nh.param("robot_frame", robot_frame_, robot_frame_ );

	robot_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>( "gnss_robot_pose", 1 );
	gnss_subscriber_ = nh.subscribe( "move_base_simple/goal", 100, &GnssFilter::doFiltering, this );
}/*}}}*/

void GnssFilter::doFiltering( const geometry_msgs::PoseStamped::ConstPtr& msg )
{/*{{{*/
	ROS_INFO( "do filtering" );
}/*}}}*/

void GnssFilter::updatePoselist()
{/*{{{*/
  static geometry_msgs::TransformStamped transform;
  static geometry_msgs::TransformStamped transform_past;
	static bool is_first_time = false;

	// get odom -> robot tf
	try {
		transform = tf2_buffer_.lookupTransform( odom_frame_, robot_frame_, ros::Time(0), ros::Duration( 0.1 ) );
		is_first_time = true;
	}
	catch ( tf2::TransformException &ex ) {
		ROS_INFO( "[GnssFilter] cannot lookup tf ( from %s to %s )", odom_frame_.c_str(), robot_frame_.c_str() );
		ros::Duration(1.0).sleep();
		return;
	}

	if ( is_first_time ) return;
	
	transform_past = transform;
}/*}}}*/

void GnssFilter::spin()
{/*{{{*/
	ros::spin();
}/*}}}*/
