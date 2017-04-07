#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <cmath>


using namespace std;

#define NODE_NAME "getplan"
#define TOPIC_NAME "/move_base/make_plan"

move_base_msgs::MoveBaseGoal * createGoal( int, int, float );
string testState( actionlib::SimpleClientGoalState );

int main(int argc,char **argv) {

    ros::init( argc, argv, NODE_NAME );
    ros::NodeHandle nh;

    ros::ServiceClient client =
       nh.serviceClient<nav_msgs::GetPlan>( TOPIC_NAME );
    client.waitForExistence();
    
    
    /* INITIALIZATION */
    nav_msgs::GetPlan plannermsg;

    plannermsg.request.start.header.frame_id = "map";
    plannermsg.request.start.header.stamp = ros::Time::now();
    /* FILL THE REST FOR THE START LOCATION */
    float theta = 0 * M_PI / 180 ;
    plannermsg.request.start.pose.position.x = 0;
    plannermsg.request.start.pose.position.y = 0;
    plannermsg.request.start.pose.position.z = 0;
    plannermsg.request.start.pose.orientation.x = 0;
    plannermsg.request.start.pose.orientation.y = 0;
    plannermsg.request.start.pose.orientation.z = sin( theta/2 );
    plannermsg.request.start.pose.orientation.w = cos( theta/2 );


    plannermsg.request.goal.header.frame_id = "map";
    plannermsg.request.goal.header.stamp = ros::Time::now();
    /* FILL THE REST FOR THE GOAL LOCATION */
    theta = 90 * M_PI / 180 ;
    plannermsg.request.start.pose.position.x = 1;
    plannermsg.request.start.pose.position.y = 1;
    plannermsg.request.start.pose.position.z = 0;
    plannermsg.request.start.pose.orientation.x = 0;
    plannermsg.request.start.pose.orientation.y = 0;
    plannermsg.request.start.pose.orientation.z = sin( theta/2 );
    plannermsg.request.start.pose.orientation.w = cos( theta/2 );


    /* CALL SERVICE CHECK FOR RESULTS, ETC */
    if ( client.call( plannermsg ) ) {
	
        ROS_INFO_STREAM("Service successfully called");
        // ROS_INFO_STREAM("Status_message:" << plannermsg.response );

        for( int n = 0; n < plannermsg.response.plan.poses.size(); n++ )
          ROS_INFO_STREAM( plannermsg.response.plan.poses[n] );
    
    } else {
	
        ROS_ERROR_STREAM("Error while calling plannermsg");
        // ROS_ERROR_STREAM("Status_message:" << plannermsg.response );
    
    }


    // create goal to send to the bot
    // this goal fails because theta is 0.0:
    // [ERROR] [1488523621.229798856, 15.870000000]: Quaternion has length close to zero... discarding as navigation goal
    // [ERROR] [1488523630.631711562, 25.230000000]: Quaternion has length close to zero... discarding as navigation goal
    // move_base_msgs::MoveBaseGoal *goal = createGoal( 1, 1, 0.0 );

    // // send goal to bot and wait for it to execute
    // ac.sendGoal( *goal );
    // ac.waitForResult();

    // // print out result
    // ROS_INFO_STREAM( testState( ac.getState() ) );
    // delete goal;


    // // new goal
    // goal = createGoal( 1, 2, 90.0 );
    // ac.sendGoal( *goal );
    // ac.waitForResult();
    // ROS_INFO_STREAM( testState( ac.getState() ) );
    // delete goal;


    // // new goal
    // goal = createGoal( 0, 0, -90.0 );
    // ac.sendGoal( *goal );
    // ac.waitForResult();
    // ROS_INFO_STREAM( testState( ac.getState() ) );
    // delete goal;


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