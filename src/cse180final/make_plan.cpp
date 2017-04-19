// http://wiki.ros.org/map_server?distro=kinetic
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>

using namespace std;


#define NODE_NAME "make_plan"
// the occupancy grid of the map
#define TOPIC_MAP "/map"
// map metadata
#define TOPIC_MAP_META "/map_metadata"


bool DEBUGGING = true;

// mapCallback function
void mapCallback( const nav_msgs::OccupancyGrid & msg );
// mapMetaCallback function
void mapMetaCallback( const nav_msgs::MapMetaData & msg );


int main( int argc, char ** argv ) {

  	// init ros
  	ros::init( argc, argv, NODE_NAME );
  	ros::NodeHandle nh;

	// create the Subscriber and set the topic to listen to
	// ros::Subscriber sub_map = nh.subscribe( TOPIC_MAP, 1000, &mapCallback );
	ros::Subscriber sub_map_meta = nh.subscribe( TOPIC_MAP_META, 1000, &mapMetaCallback );

	// listen to topic
	ros::spin();

	return 0;

}

// get map's occupancy grid, -1 - unkown 0 - 100 p of occupancy
// http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
void mapCallback( const nav_msgs::OccupancyGrid & msg ) {

	if( DEBUGGING ) {
		
		ROS_ERROR_STREAM( setprecision(2) << msg );
		
	}

}

// get map meta data of the form:
// http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html
/**
[ERROR] [1492465743.173240642, 2576.940000000]: map_load_time: 1986.160000000
resolution: 0.05
width: 1984
height: 1984
origin: 
  position: 
    x: -50
    y: -50
    z: 0
  orientation: 
    x: 0
    y: 0
    z: 0
    w: 1
**/
void mapMetaCallback( const nav_msgs::MapMetaData & msg ) {

	if( DEBUGGING ) {
		
		ROS_ERROR_STREAM( setprecision(2) << msg );
		
	}

}


