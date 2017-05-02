// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include <utility>

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

    const float RANGE_MAX = 5.0; // the range limit we are setting for the laser
	const float RANGE_MIN = 0.70;
	const float ERROR = 0.10;
	const int HALF = msg.ranges.size() / 2;
	const float INCREM = (float)msg.angle_increment;
	//const float PI270 =  (float)msg.angle_max * 2;
	const float POST_RADIUS = 0.07;
	float prior_range = (float)msg.ranges[0];
	float min_range = 0.0;
	float max_range_right = 0.0;
    float max_range_left = 0;
	float range = 0.0;
	float angle = 0.0;
	float radius_calc = 0.0;
	int half_width = 0;
	int width = 0;
	vector<float> ranges;
// 	vector< Pair<float, float> > profiles;
	for( int n = 0; n < msg.ranges.size(); n++ ) {

		range = (float)msg.ranges[n];
        
        // bad range
        if( range > RANGE_MAX || range < RANGE_MIN  ) {
            
            // reset
            max_range_left = 0;
            min_range = 0;
            width = 0;
            half_width = 0;
            continue;
        
        } else if( max_range_left == 0 )
        
            max_range_left = range;
            
        else if( max_range_left - range > 0 )
        
            width++;
            
        else {
         
            if( min_range == 0 ) {
            
                min_range = range;
                half_width = width;
                
            }
            
            width++;
            
            // check for post
            if( half_width <= width/2 )
                ROS_INFO( "POST_DETECTED" );
            
        }
    
        /**
        ROS_INFO( "Range: [%.6f]", range );
        
        ROS_INFO( "Max1: [%.6f], Max2: [%.6f], Min: [%.6f], Width: [%d], Half: [%d]", 
	        max_range_left,
	        max_range_right, 
	        min_range,
	        width,
	        half_width
    	);
		    	
		// stay in the bubble
        if( max_range_left > RANGE_MAX || max_range_left < RANGE_MIN ) {
            
            max_range_left = range;
            min_range = range;
            ROS_INFO( "new max" );   
            
        // left side of porabola
        } else if( max_range_left > range && min_range > range) {
            
            width++;
            half_width = width;
            min_range = range;
            ROS_INFO( "left" );
            
        // right side of porabola
        } else if( min_range < range && half_width > 0 ) {
        
            width++;
            half_width--;
            ROS_INFO( "right" );
        
        // found 2nd maximum
        } else if( min_range < range && half_width == 0 ) {
            
            max_range_right = range;
            //if( isPost( min_range, width-1 ) )
            if( true )
                ROS_INFO( "post detected" );
		    
		    // reset
            max_range_left = 0;
            width = 0;
            half_width = 0;
            
        } else {
            
            ROS_INFO( "else reset" );
            // reset
            max_range_left = range;
            min_range = range;
            width = 0;
            half_width = 0;
            
        }
            **/

        /**

        // limit our scan for posts to certain radius
		if( prior_range > RANGE_LIMIT && range <= RANGE_LIMIT ) {
			
			prior_range = range;
			ranges.push_back( range );		
			continue;

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

			ranges.clear();
			width = 0;

		} else {

			ranges.clear();
			width = 0;	

		}
        **/
        
	}


}




