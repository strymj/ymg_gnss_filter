#include <ymg_gnss_filter/gnss_filter.hpp>

int main ( int argc, char **argv )
{/*{{{*/
	ros::init( argc, argv, "gnss_filter_node" );
	ros::NodeHandle nh;

	GnssFilter gf;
	gf.spin();

	return 0;
}/*}}}*/
