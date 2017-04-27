// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>

using namespace std;


#define NODE_NAME "detectpost"
#define TOPIC_NAME "/scan"

bool DEBUGGING = true;
const int UNDEFINED = -1;

struct Post {
	int start_index;
	int stop_index;
	int index_size;
	float range;
	float x;
	float y;
	float angle;
	
	Post() {
		start_index = 0;
		stop_index = 0;
		x = 0;
		y = 0;
		angle = 0;
	}

	Post( int start, int stop, int zero_count ) {
		start_index = start;	
		stop_index = stop;
		index_size = zero_count;
	}

};


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


const int MIN_POST_ZEROS = 150; // 167 is the real number
vector<Post *> posts;
float ANGLE_CONVERTER = 0.375;
void callback( const sensor_msgs::LaserScan & msg ) {

	if( DEBUGGING ) {
		
		// ROS_INFO( setprecision(2) << msg );
		
	}

    	// loop through intensities to detect post signatures
	int zero_count = 0;
	int start_index = NULL;
	int stop_index = NULL;
	vector<Post *> recent_posts;
	for( int n = 0; n < msg.intensities.size(); n++ ) {
	    
		if( (float)msg.intensities[n] == 0.0 ) {

			zero_count++;			
			if( start_index == NULL )
				start_index = n;

		} else if( zero_count >= MIN_POST_ZEROS ) {

			recent_posts.push_back( new Post( start_index, stop_index, zero_count ) );
			zero_count = 0;
			start_index = NULL;			
			stop_index = NULL;
		
		} else {

			zero_count = 0;
			start_index = NULL;			
			stop_index = NULL;
			
		}		

	    
	}


	// calculate position of recently discovered posts
	int index, range;
	float angle;
	for( int n = 0; n < recent_posts.size(); n++ ) {

		index = (recent_posts[n]->index_size / 2) + recent_posts[n]->start_index;
		range = msg.ranges[ index ];

		// calculate angle
		if( index < 360 ) {

			angle = 135 - (index * ANGLE_CONVERTER);

		} else if( index > 360 ) {

			angle = ANGLE_CONVERTER * (index - 360);

		} else {

			angle = 0;
		
		}

		// calculate x, y position of post
		// TODO: need to use frame transform matrices to get position of post
		// TODO: right now it is relative to robot but we need it relative to map
		// TODO: [POST][ROBOT][MA] = [POST_ACTUAL]	
		recent_posts[n]->x = range * cosf(angle);
		recent_posts[n]->y = range * sinf(angle);;
		recent_posts[n]->range = range;
		recent_posts[n]->angle = angle;
		ROS_INFO( 
			"Post: ( [%f], [%f]), Range: [%f], Angle: [%f]",
			recent_posts[n]->x,
			recent_posts[n]->y,
			range,
			angle	 
		);
		//ROS_ERROR_STREAM( (*recent_posts[n]) );


	}	
	

	if( recent_posts.size() == 0 )
		ROS_ERROR_STREAM( "no posts detected" );

}




