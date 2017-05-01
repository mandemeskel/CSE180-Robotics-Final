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

    const float RANGE_LIMIT = 5.0; // the range limit we are setting for the laser
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
	int width = 0;
	vector<float> ranges;
	for( int n = 1; n < msg.ranges.size(); n++ ) {

		range = (float)msg.ranges[n];
		//index = n;

        // limit our scan for posts to certain radius
// 		if( isinf( prior_range ) && !isinf( range ) ) {
		if( prior_range > RANGE_LIMIT && range <= RANGE_LIMIT ) {
			
			prior_range = range;
			ranges.push_back( range );		
			continue;

// 		} else if ( isinf( prior_range ) && isinf( range ) ) {
        } else if ( prior_range > RANGE_LIMIT && range > RANGE_LIMIT ) {

			continue;		

		}
		
		if( range < prior_range ) {
			
			ranges.push_back( range );
			width++;

		} else if( ranges.size() > 0 ) {

			angle = abs( (HALF - n) * INCREM ); // relative front of robot
			radius_calc = ranges[0] * sinf( angle );
			//ROS_INFO( "Angle: [%.6f], Half: [%d], n: [%d], Increm: [%.6f]", angle, HALF, n, INCREM );

			ROS_INFO( "Range1: [%.6f], Width: [%d], Radius Calc: [%.6f], Angle: [%.6f]", 
			    ranges[0],
			    width, 
			    radius_calc,
			    angle 
			);			

			
// 			ROS_INFO( "PriorRange: [%.6f], Range1: [%.6f], Rangei: [%.6f], RangeDelta: [%.6f], Radius Calc: [%.6f], Angle: [%.6f]", prior_range, ranges[0], ranges[ranges.size()-1], ranges[0] - ranges[ranges.size()-1], radius_calc,
// angle );			

/**
			if( ((radius_calc * ERROR) + radius_calc) >= POST_RADIUS && ((radius_calc * ERROR) - radius_calc) <= POST_RADIUS ) {
				
				ROS_INFO( "PriorRange: [%.6f], Range1: [%.6f], Rangei: [%.6f], Radius Calc: [%.6f], Angle: [%.6f]", prior_range, ranges[0], ranges[ranges.size()-1], radius_calc,
angle );				

			}
**/
			ranges.clear();
			width = 0;

		} else {

			ranges.clear();
			width = 0;	

		}

	}


}




