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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


using namespace std;
using namespace message_filters;
using namespace nav_msgs;


#define NODE_NAME "navigatepoints"
#define TOPIC_NAME "/move_base"

// current husky position
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D priorLocation;

// for getting next random position
random_numbers::RandomNumberGenerator* rng = new random_numbers::RandomNumberGenerator();

// if obstacle is in front of husky, stop it
bool stop = true;

// check if the current location has been updated
bool current_location_fresh = false;

// publisher
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// goal
move_base_msgs::MoveBaseGoal * currentGoal;

move_base_msgs::MoveBaseGoal * createGoal( geometry_msgs::Pose2D );
move_base_msgs::MoveBaseGoal * createGoal( int, int, float );
move_base_msgs::MoveBaseGoal * createGoal();
string testState( actionlib::SimpleClientGoalState );
void laserHandler( const sensor_msgs::LaserScan & );
//void odometryHandler(const nav_msgs::Odometry::ConstPtr & );
void odometryHandler(const nav_msgs::Odometry & );
void moveHusky( move_base_msgs::MoveBaseGoal * );


int main(int argc,char **argv) {

    ros::init( argc, argv, NODE_NAME );
    ros::NodeHandle nh;
    
    // husky controller
    MoveBaseClient ac( TOPIC_NAME, true );

    ROS_INFO_STREAM("Waiting for server to be available...");
    while ( !ac.waitForServer() ) {

    }
    ROS_INFO_STREAM("done!");

    // get current location
    // nh.subscribe( "/odometry/filtered", 1, odometryHandler );
    message_filters::Subscriber<Odometry> odom_sub( nh, "/odometry/filtered", 10 );    
    odom_sub.registerCallback( odometryHandler );

    // look for obstacles
    // nh.subscribe( "/scan", 10, laserHandler );
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub( nh, "/scan", 10 );
    laser_sub.registerCallback( laserHandler );

    // new goal
    while( true ) {
        
        ros::spinOnce();

        if( !stop && current_location_fresh ) {

            currentGoal = createGoal();

            // copy last safe position
            priorLocation.x  = currentLocation.x;
            priorLocation.y  = currentLocation.y;
            priorLocation.theta  = currentLocation.theta;

            moveHusky( currentGoal );
            // ac.sendGoal( *currentGoal );
            // wait for 15 secs for goal to be finished
            // ac.waitForResult( ros::Duration( 15.0 ) );
            // ROS_INFO_STREAM( testState( ac.getState() ) );
            delete currentGoal;
        
        }
        
    }
    
    return 0;

}


void moveHusky( move_base_msgs::MoveBaseGoal * goal ) {
   
    MoveBaseClient ac( TOPIC_NAME, true );
    ac.sendGoal( *goal );
    // wait for 15 secs for goal to be finished
    ac.waitForResult( ros::Duration( 15.0 ) );
    ROS_INFO_STREAM( testState( ac.getState() ) );
    // delete goal;

}

move_base_msgs::MoveBaseGoal * createGoal( geometry_msgs::Pose2D pose ) {
    
    move_base_msgs::MoveBaseGoal * goal = new move_base_msgs::MoveBaseGoal();

    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();
    
    goal->target_pose.pose.position.x = pose.x;
    goal->target_pose.pose.position.y = pose.y;
    goal->target_pose.pose.orientation.w = pose.theta;

    // update state
    current_location_fresh = false;    

    return goal;

}

move_base_msgs::MoveBaseGoal * createGoal( int x, int y, float angle ) {
    
    move_base_msgs::MoveBaseGoal * goal = new move_base_msgs::MoveBaseGoal();

    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();
    
    goal->target_pose.pose.position.x = x;
    goal->target_pose.pose.position.y = y;
    goal->target_pose.pose.orientation.w = angle;

    // update state
    current_location_fresh = false;    

    return goal;

}


move_base_msgs::MoveBaseGoal * createGoal() {
    
    move_base_msgs::MoveBaseGoal * goal = new move_base_msgs::MoveBaseGoal();

    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();

    // randomly generated goal
    // TODO: check with global_costmap to see if this is an open cell
    float theta = rng->gaussian(currentLocation.theta, 0.1);
    goal->target_pose.pose.position.x = currentLocation.x + (0.75 * cos( theta ));
    goal->target_pose.pose.position.y = currentLocation.y + (0.75 * sin( theta ));
    goal->target_pose.pose.orientation.w = theta; // TO DEGREES

    ROS_INFO( 
        "Goal, x: [%.6f], y: [%.6f], theta: [%.6f]",
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.orientation.w
    );

    ROS_INFO( 
        "Current, x: [%.6f], y: [%.6f], theta: [%.6f]",
        currentLocation.x,
        currentLocation.y,
        currentLocation.theta
    );

    // update state
    current_location_fresh = false;    

    return goal;

}


string testState( actionlib::SimpleClientGoalState state ) {

    string out;

    if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
	    out = "Success";
    else
	    out = "Failure";

    return out;

}

/**
void odometryHandler(const nav_msgs::Odometry::ConstPtr & message) {

    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;

    // update state
    current_location_fresh = true;

}
**/

void odometryHandler(const nav_msgs::Odometry & message) {

    //Get (x,y) location directly from pose
    currentLocation.x = message.pose.pose.position.x;
    currentLocation.y = message.pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(
        message.pose.pose.orientation.x, 
        message.pose.pose.orientation.y, 
        message.pose.pose.orientation.z, 
        message.pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;

    // update state
    current_location_fresh = true;

}

// msg.ranges.size() - 200
const unsigned int max_angle = 520;
void laserHandler( const sensor_msgs::LaserScan & msg ) {


    float range = 0.0;
    float new_theta = 0.0;
    unsigned int n;
    for( n = 200; n < max_angle; n++ ) {

        range = (float) msg.ranges[n];

        if( range > 0.75 ) continue;

        // obstacle           
        ROS_INFO( 
            "obstacle found range: [%.6f], angle: [%.6f]",
            range,
            currentLocation.theta 
        );

        // stop husky
        stop = true;

        // move the husky back to last safe position
        // turn husky by 4degs left and right
        if( n > 360 )
            new_theta = currentLocation.theta + 0.261799;// 0.0698132
        else
            new_theta = currentLocation.theta - 0.261799; // 0.0698132

        move_base_msgs::MoveBaseGoal * temp = createGoal( 
            currentLocation.x,
            currentLocation.y,
            new_theta
        );
        ROS_INFO( 
            "Goal, x: [%.6f], y: [%.6f], theta: [%.6f]",
            temp->target_pose.pose.position.x,
            temp->target_pose.pose.position.y,
            temp->target_pose.pose.orientation.w
        );
        moveHusky( temp );
        delete temp;

    }

    if( n  >= max_angle )
        stop = false;

}





