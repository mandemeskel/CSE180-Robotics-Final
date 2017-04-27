// http://wiki.ros.org/map_server?distro=kinetic
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <typeinfo>
#include <fstream>

using namespace std;


#define NODE_NAME "make_plan"
// the occupancy grid of the map
// #define TOPIC_MAP "/move_base/global_costmap/costmap"
#define TOPIC_MAP "map"
// map metadata
#define TOPIC_MAP_META "/map_metadata"


bool DEBUGGING = true;
int * cost_map;


// mapCallback function
void mapCallback( const nav_msgs::OccupancyGrid & msg );
// mapMetaCallback function
void mapMetaCallback( const nav_msgs::MapMetaData & msg );


int main( int argc, char ** argv ) {

  	// init ros
  	ros::init( argc, argv, NODE_NAME );
  	ros::NodeHandle nh;

	// create the Subscriber and set the topic to listen to
	ros::Subscriber sub_map = nh.subscribe( TOPIC_MAP, 1000, &mapCallback );
	// ros::Subscriber sub_map_meta = nh.subscribe( TOPIC_MAP_META, 1000, &mapMetaCallback );

	// listen to topic
	ros::spin();

	return 0;

}

// get map's occupancy grid, -1 - unkown 0 - 100 p of occupancy
// http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
void mapCallback( const nav_msgs::OccupancyGrid & msg ) {

	// if( DEBUGGING ) {
		
	// 	ROS_ERROR_STREAM( setprecision(2) << msg );
		
	// }

	int width = msg.info.width;
	int height = msg.info.height;
	int size = width * height;
	bool is_first_open = false;
	// First map cell: 1582129
	// End map cell: 2387754
	int first_open = 0;
	int last_open = 0;
	int unknowns = 0;


	if( DEBUGGING ) {

		ROS_ERROR_STREAM( setprecision(2) << width );
		ROS_ERROR_STREAM( setprecision(2) << height );
		// 3936256 size of msg.data
		ROS_ERROR_STREAM( setprecision(2) << msg.data.size() );

	}

	// get first and last cells of map
	for( int y = 0; y < size; y++ ) {

		if( (int)msg.data[y] != -1 && !is_first_open && first_open == 0 ) {

			is_first_open = true;
			first_open = y;

		} else if ( (int)msg.data[y] != -1 && !is_first_open ) {

			is_first_open = true;
		
		} else if(  (int)msg.data[y] != -1 && is_first_open ) {

			last_open = y;
			is_first_open = false;

		}

	}

	if( DEBUGGING )
		ROS_ERROR_STREAM( "first " << first_open << " last " << last_open );

	// create our copy of the costmap
	if( cost_map == NULL )
		cost_map = new int[last_open - first_open];

	// ofstream file;
	// file.open( "map.txt" );

	int x = 0;
	for( int y = first_open; y <= last_open; y++ ) {
	
		if( (int)msg.data[y] == -1 ) 
			unknowns++;
		else if( unknowns > 20 )
			unknowns = 0;

		if( unknowns < 20 ) {

			// ROS_ERROR_STREAM( "Cell: " << y << " " << (int)msg.data[y] );
			// file << "Cell: " << y << " " << (int)msg.data[y] << endl;

			// add cells to cost_map
			cost_map[x] = (int)msg.data[y];
			x++;

		}

	}

	// file.close();

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


