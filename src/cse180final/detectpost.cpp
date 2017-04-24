// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

using namespace std;


#define NODE_NAME "laser_scan_listner"
#define TOPIC_NAME "/scan"

bool DEBUGGING = true;

// callback function
void callback( const sensor_msgs::LaserScan & msg );


int main( int argc, char ** argv ) {

  	// init ros
  	ros::init( argc, argv, NODE_NAME );
  	ros::NodeHandle nh;

	  // create the Subscriber and set the topic to listen to
	  ros::Subscriber sub = nh.subscribe( TOPIC_NAME, 1000, &callback );

	  // listen to topic
	  ros::spin();

	return 0;

}


void callback( const sensor_msgs::LaserScan & msg ) {

	if( DEBUGGING ) {
		
		ROS_ERROR_STREAM( setprecision(2) << msg );
		
	}

}




