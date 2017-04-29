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

	ROS_INFO( "detectpost started" );

	// listen to topic
	ros::spin();

	return 0;

}


const int MIN_POST_ZEROS = 30; // number 0.0 for a post in msg.intensities
vector<Post *> posts;
float ANGLE_CONVERTER = 0.375;
double POST_INTENSITY = 3.1092664570587305;
void callback( const sensor_msgs::LaserScan & msg ) {

	if( DEBUGGING ) {
		
		// ROS_INFO( setprecision(2) << msg );
		
	}

	const float ERROR = 0.10;
	const int HALF = msg.ranges.size() / 2;
	const float INCREM = (float)msg.angle_increment;
	//const float PI270 =  (float)msg.angle_max * 2;
	const float POST_RADIUS = 0.07;
	float prior_range = (float)msg.ranges[0];
	float range = 0.0;
	float angle = 0.0;
	float radius_calc = 0.0;
	//int index = 0;
	//int width = 0;
	vector<float> ranges;
	for( int n = 1; n < msg.ranges.size(); n++ ) {

		range = (float)msg.ranges[n];
		//index = n;

		if( isinf( prior_range ) && !isinf( range ) ) {
			
			prior_range = range;
			ranges.push_back( range );		
			continue;

		} else if ( isinf( prior_range ) && isinf( range ) ) {

			continue;		

		}
		
		if( range < prior_range ) {
			
			ranges.push_back( range );
			//width++;

		} else if( ranges.size() > 0 ) {

			angle = abs( (HALF - n) * INCREM ); // relative front of robot
			radius_calc = ranges[0] * sinf( angle );
			//ROS_INFO( "Angle: [%.6f], Half: [%d], n: [%d], Increm: [%.6f]", angle, HALF, n, INCREM );
			
			ROS_INFO( "PriorRange: [%.6f], Range1: [%.6f], Rangei: [%.6f], Radius Calc: [%.6f], Angle: [%.6f]", prior_range, ranges[0], ranges[ranges.size()-1], radius_calc,
angle );			

/**
			if( ((radius_calc * ERROR) + radius_calc) >= POST_RADIUS && ((radius_calc * ERROR) - radius_calc) <= POST_RADIUS ) {
				
				ROS_INFO( "PriorRange: [%.6f], Range1: [%.6f], Rangei: [%.6f], Radius Calc: [%.6f], Angle: [%.6f]", prior_range, ranges[0], ranges[ranges.size()-1], radius_calc,
angle );				

			}
**/
			ranges.clear();
			//width = 0;

		} else {

			ranges.clear();
			//width = 0;	

		}

	}


/**	
	double intns = 0.0;
	for( int n = 0; n < msg.intensities.size(); n++ ) {

		intns = msg.intensities[n] * 1e18;
		if( intns < POST_INTENSITY + 0.15 || intns > POST_INTENSITY - 0.15 ) {

			ROS_INFO( "Range: [%.6f], Intensity: [%f], Expected: [%f], Index: [%d]", (float)msg.ranges[n], intns, POST_INTENSITY, n );

		}

	}
**/


/**
    	// loop through intensities to detect post signatures
	int zero_count = 0;
	int start_index = NULL;
	int stop_index = NULL;
	vector<Post *> recent_posts;
	//vector<float> intensities;
	vector<float> ranges;
	for( int n = 0; n < msg.intensities.size(); n++ ) {

		if( (float)msg.intensities[n] == POST_INTENSITY ) {

			intensities.push_back( n );
			ranges.push_back( (float)msg.ranges[n] );
			// ROS_INFO( "Range: [%.6f], Index: [%.6f]", (float)msg.ranges[n], n );

			zero_count++;			
			if( start_index == NULL )
				start_index = n;

		} else if( zero_count >= MIN_POST_ZEROS ) {
			
			stop_index = n;
			recent_posts.push_back( new Post( start_index, stop_index, zero_count ) );
			//ROS_INFO( "Post Detected" );
			zero_count = 0;
			start_index = NULL;			
			stop_index = NULL;
		
		} else {

			//ROS_INFO( "False Post Detected" );
			zero_count = 0;
			start_index = NULL;			
			stop_index = NULL;
			
		}		

	    
	}

**/

/**
	for( int n = 0; n < ranges.size(); n++ ) {

		ROS_INFO( "Range: [%.6f], Index: [%.6f]", ranges[n], intensities[n] );		

	}
**/
	
	// calculate position of recently discovered posts
	/**int index;
	float range, angle;
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
		recent_posts[n]->y = range * sinf(angle);
		recent_posts[n]->range = range;
		recent_posts[n]->angle = angle;

		ROS_INFO( 
			"Post: ( [%.6f], [%.6f]), Range: [%.6f], Angle: [%.6f], Index: [%d], Start: [%d], Index_Size: [%d], Expected Width: [%.6f]",
			recent_posts[n]->x,
			recent_posts[n]->y,
			range,
			angle,
			index,
			recent_posts[n]->start_index,
			recent_posts[n]->index_size,
			2 * range * sinf(angle)
		);

	}	
	**/

	//if( recent_posts.size() == 0 )
	//	ROS_ERROR_STREAM( "no posts detected" );

}




