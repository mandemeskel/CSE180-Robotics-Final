#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc,char **argv) {

    ros::init(argc,argv,"movearound");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
	ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = 1;
    goal.target_pose.pose.position.y = 1;
    goal.target_pose.pose.orientation.w = 1.0;

    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	ROS_INFO_STREAM("Success");
    else
	ROS_INFO_STREAM("Failure");
    
    return 0;    
}
