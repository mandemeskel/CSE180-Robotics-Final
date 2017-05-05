#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_listener.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <vector>
#include <cmath>
#include <sstream>


using namespace std;
using namespace geometry_msgs;


#define NODE_NAME "randomnav"

bool DEBUGGING = true;

// state of husy
enum State {
    TURNING, // husky is turning towards the goal
    PURSING, // husky is pursing the goal
    COMPLETE, // the goal is complete
    OBSTACLE // there is an obstacle and we are trying to avoid
} current_state = COMPLETE;

// linear speed of husky, 1m/s
const float LINEAR_SPEED = 0.5;

// angular speed of husky, 15deg/s
const float ANGULAR_SPEED = 0.261799;

// the minimum angle to avoid obstacle, in LaserScan::ranges index
const unsigned int MIN_ANGLE = 260;

// the maximum angle to avoid obstacle, in LaserScan::ranges index
const unsigned int MAX_ANGLE = 460;

// minimum desired free space range to consider for exploration
const float DESIRED_RANGE = 7.0;

// mind your personal space, any obstacles less than this will trigger
// obstacle avoidance
const float TOO_CLOSE = 1; // can be 0.75 but will cause a lot of close calls

// don't go outside the map, don't just drive in a straight line for
// a prolonged period of time
const float MAX_RANGE = 20.0;

// twist publisher, for obstacle avoidance
ros::Publisher twist_pub;

// for getting next random position
random_numbers::RandomNumberGenerator* rng = new random_numbers::RandomNumberGenerator();

// do we need to publish a twist msg to husky?
bool publish = false;

// the twist message to publish to cmd_vel
Twist twist_msg;

// the goal range we are pursing
float goal_range = 0.0;

// the LaserSan::Ranges index of goal range, we use this
// as angle to tell husky which way to turn when avoiding
// obstacles, htis prevents a back and forth stuck turning scenario
int goal_index = 0;

// topic callbacks
void laserHandler( const sensor_msgs::LaserScan & );
// returns the radians relative to robot that 
// the passed range index has
float getDirection( int );
// checks if y is in error of x, ( x, y, error )
bool inError( float, float, float );
// send twist commands to cmd_vel, ( linear, angular )
void moveHusky( float, float );


int main(int argc,char **argv) {

    ros::init( argc, argv, NODE_NAME );
    ros::NodeHandle nh;

    ROS_INFO( "randomnav started!" );

    // publish to cmd_vel to send dirct messages to avoid obstacles
    twist_pub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );

    // look for obstacles and set goal
    ros::Subscriber laser_sub = nh.subscribe( "scan", 1, &laserHandler );
    // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub( nh, "/scan", 100 );
    // laser_sub.registerCallback( laserHandler );

    ROS_INFO( "randomnav spinning..." );
    
    // set publisher rate
    ros::Rate rate( 1 );

    while( ros::ok() ) {

        ros::spinOnce();

        if( publish || current_state == PURSING ) {

            // if( DEBUGGING )
            //     ROS_INFO_STREAM( "publish, twist_msg: " << twist_msg );

            twist_pub.publish( twist_msg );
            publish = false;

        }

    }
    
    return 0;

}


void laserHandler( const sensor_msgs::LaserScan & msg ) {

    // if( DEBUGGING )
    //     ROS_INFO( "laserHandler, current_state: [%d]", current_state );

    float range = 0.0;
    float new_theta = 0.0;
    unsigned int largest_range = 0;
    unsigned int n = 0;
    vector<float> directions;
    vector<float> ranges;
    
    // we are turning towards a new goal, let's if it is in front of us
    if( current_state == TURNING ) {

        for( int m = 350; m < 370; m++ ) {

            range = (float) msg.ranges[m];

            // are we facing the right direction 5% laser error
            if( inError( goal_range, range, 0.01 ) ) {

                // move forward 1m/s
                moveHusky( LINEAR_SPEED, 0 );
                // update state, we are pursing a goal :D
                current_state = PURSING;

                if( DEBUGGING )
                    ROS_INFO( "laserHandler, pursing: [%.6f], angle: [%.6f], index: [%d]", range, getDirection( m ), m );

            }

        } 

        // keep spinning if we haven't yet started pursing goal
        if( current_state == TURNING )
            publish = true;

    // continue
    } else {

        for( n = 0; n < msg.ranges.size(); n++ ) {

            range = (float) msg.ranges[n];

            // ignore big open ranges
            if( range > MAX_RANGE || isinf(range) ) continue;

            // find direction to wander in, next goal 
            if( current_state == COMPLETE ) {

                if( range > DESIRED_RANGE ) {
                    // directions.push_back( getDirection( n ) );
                    ranges.push_back( n );
                }

                // back up if there are no locations around greater
                // than our disered range
                if( range > (float) msg.ranges[ largest_range ] && range > TOO_CLOSE )
                    largest_range = n;

            }

            // collision detection and avoidance
            // only for obstacles in front of the husky
            // 75deg cone in fornt of the husky
            if( n > MIN_ANGLE || n < MAX_ANGLE ) {

                if( range > TOO_CLOSE ) continue;

                // obstacle           
                ROS_INFO( 
                    "obstacle found range: [%.6f], angle: [%.6f] - index: [%d]",
                    range,
                    getDirection( n ),
                    n
                );

                // new_theta = ANGULAR_SPEED;
                // turn husky to the left or right to avoid obstacle
                if( n > 360 || goal_index > 360 )
                    new_theta = ANGULAR_SPEED;
                else
                    new_theta -= ANGULAR_SPEED;

                // send twist message to stop and turn away from collision
                moveHusky( 0, new_theta );
                
                // update state
                current_state = OBSTACLE;

                break;

            }

        }

    }

    // if( DEBUGGING )
    //     ROS_INFO( "laserHandler, ranges: [%d]", (int)ranges.size() );

    // find a random goal
    if( current_state == COMPLETE ) { 

        // int goal_index = 0;
        if( ranges.size() != 0 ) {

            int sample_size = ranges.size();
            float random_num = abs( rng->gaussian( (double)sample_size, sample_size/2 ) );
            int rng_index = ((int)random_num) % (sample_size - 1);
            goal_index = ranges[ rng_index ];

            if( DEBUGGING )
                ROS_INFO_STREAM( "laserHandler, RNG number: " << random_num << " rng index: " << rng_index << " sample size: " << sample_size );

        } else {

            goal_index = largest_range;

        }
        goal_range = msg.ranges[ goal_index ];

        // stop and spin until we are facing the goal range
        // spin 1 degree at a time
        if( goal_index > 360 )
            moveHusky( 0, -ANGULAR_SPEED );
        else
            moveHusky( 0, ANGULAR_SPEED );

        // update state
        current_state = TURNING;
    
        if( DEBUGGING )
            ROS_INFO( "laserHandler, goal_range: [%.6f], goal_index: [%d]", goal_range, goal_index );

    }

    // if we go through ranges without detecting a an obstacle
    // and we are in obstacle avoidance state, update the state
    if( n  >= msg.ranges.size() && current_state == OBSTACLE ) {

        // stop turning, we're free from danger
        moveHusky( 0, 0 );
        // update state
        current_state = COMPLETE;
        if( DEBUGGING )
            ROS_INFO( "laserHandler, obstacle free" );

    }

}


// the angle increment of the laserscanner in radians
const float INCREM = 0.0065540750511;
// return the radians of the lasercan range index passed
float getDirection( int index ) {

    float radians = 0.0;

    // 360 is the half way point of the ranges array
    // exactly infront of the husky
    radians = (index - 360) * INCREM;

    return radians;

}



// sends a twist message cmd_vel
void moveHusky( float x, float angular ) {

    // geometry_msgs::Twist msg;
    // msg.linear.x = x;
    // msg.angular.z = angular;
    // twist_pub.publish( msg );
    twist_msg.linear.x = x;
    twist_msg.angular.z = angular;
    publish = true;
    
    if( DEBUGGING )
        ROS_INFO_STREAM( "moveHusky: " << twist_msg );

}


// checks if y is in error of x 
bool inError( float x, float y, float error ) {
    
    float min = x - (x * error);
    float max = x + (x * error);
        
    // if( DEBUGGING )    
    //     ROS_INFO_STREAM( "inError, x: " 
    //     << x << " y: " 
    //     << y << " error: " 
    //     << error << " bool: "
    //     << (y > min && y < max) );
    
    return y > min && y < max;

}

