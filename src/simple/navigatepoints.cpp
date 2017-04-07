#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using namespace std;

#define NODE_NAME "navigatepoints"
#define TOPIC_NAME "move_base"

move_base_msgs::MoveBaseGoal * createGoal( int, int, float );
string testState( actionlib::SimpleClientGoalState );

int main(int argc,char **argv) {

    ros::init( argc, argv, NODE_NAME );
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
	ac( TOPIC_NAME, true );
    ROS_INFO_STREAM("Waiting for server to be available...");
    while ( !ac.waitForServer() ) {

    }
    ROS_INFO_STREAM("done!");

    // move_base_msgs::MoveBaseGoal goal;

    // goal.target_pose.header.frame_id = "map";
    // goal.target_pose.header.stamp = ros::Time::now();
    
    // goal.target_pose.pose.position.x = 1;
    // goal.target_pose.pose.position.y = 1;
    // goal.target_pose.pose.orientation.w = 1.0;

    // create goal to send to the bot
    // this goal fails because theta is 0.0:
    // [ERROR] [1488523621.229798856, 15.870000000]: Quaternion has length close to zero... discarding as navigation goal
    // [ERROR] [1488523630.631711562, 25.230000000]: Quaternion has length close to zero... discarding as navigation goal
    move_base_msgs::MoveBaseGoal *goal = createGoal( 1, 1, 0.0 );

    // send goal to bot and wait for it to execute
    ac.sendGoal( *goal );
    ac.waitForResult();

    // print out result
    ROS_INFO_STREAM( testState( ac.getState() ) );
    delete goal;


    // new goal
    goal = createGoal( 1, 2, 90.0 );
    ac.sendGoal( *goal );
    ac.waitForResult();
    ROS_INFO_STREAM( testState( ac.getState() ) );
    delete goal;


    // new goal
    goal = createGoal( 0, 0, -90.0 );
    ac.sendGoal( *goal );
    ac.waitForResult();
    ROS_INFO_STREAM( testState( ac.getState() ) );
    delete goal;

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	// ROS_INFO_STREAM("Success");
    // else
	// ROS_INFO_STREAM("Failure");

    return 0;

}


move_base_msgs::MoveBaseGoal * createGoal( int x, int y, float angle ) {
    
    move_base_msgs::MoveBaseGoal * goal = new move_base_msgs::MoveBaseGoal();

    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();
    
    goal->target_pose.pose.position.x = x;
    goal->target_pose.pose.position.y = y;
    goal->target_pose.pose.orientation.w = angle;

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