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
vector<int> detectPost(  const vector<float> );


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

	detectPost( msg.ranges );

}


const float MAX_RANGE = 4.0;
const float MIN_RANGE = 0.5;
const int MIN_WIDTH = 5;
const int MAX_WIDTH = 30;
vector<int> detectPost( const vector<float> ranges ) {

    bool is_post = false;
    float range = 0;
    int width = 0;
    int half_width = 0;
    float min_range = 0.0;
    float prior = 0.0;
    int pindex = 0;
    vector<int> posts;
    for( unsigned int n = 0; n < 720; n++ ) {

        range = ranges[n];

        if( range > MAX_RANGE || range < MIN_RANGE ) {

            // cout << "bad range" << endl;
            prior = 0.0;
            min_range = 0.0;
            width = 0;
            half_width = 0;	
 	    pindex = 0;
            continue;

        } else if( prior > MAX_RANGE || prior < MIN_RANGE ) {

            //cout << "left range: " << range << endl;
            prior = range;
            min_range = range;
            width = 1;
            half_width = 1;		
 	    pindex = 0;

        } else if( min_range > range ) {

           // cout << "left range: " << range << endl;
            min_range = range;
            width++;
            half_width = width;
	    pindex = n;

        } else if( min_range < range ) {

            if( half_width > 0 ) {
                
               // cout << "right range: " << range << endl;
                width++;
                half_width--;

            } else {

                //cout << "right range: " << range << endl;
                width++;
                if( width < MAX_WIDTH && width > MIN_WIDTH ) {

                    // the further away the smaller the width should be
                    if( ( width > 10 && min_range < 2 ) || ( width < 10 && min_range > 2 ) ) {
		                cout << "post detected, min-range:" << min_range << " width: " << width << endl;
				cout << pindex << endl;
				posts.push_back( pindex );
			
			}
                
                }

                prior = 0.0;
                min_range = 0.0;
                width = 0;
                half_width = 0;
		pindex = 0;

            }

        } 


    }

    return posts;
}

