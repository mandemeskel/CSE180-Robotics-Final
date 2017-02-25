#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

using namespace std;

#define NODE_NAME "drawsquare"
#define TOPIC_NAME "/turtle1/cmd_vel"

int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( TOPIC_NAME, 100 );

  // setup random number generator
  srand( time(0) );

  // set publisher rate
  ros::Rate rate( 1 );

  // number of messages
  int msgs_sent = 0;

  // the loop, this where we draw a square
  while( ros::ok() ) {

    // create msg to be spent
    geometry_msgs::Twist msg;

    // velocity, m/s
    msg.linear.x = 1;

    // angular velocity, rad/s
    if( msgs_sent % 2 == 0) {
      msg.angular.z = 1.57;
      msg.linear.x = 0;
      msgs_sent = 0;
    }

    // publish msg
    pub.publish( msg );

    msgs_sent++;

    // throttle publishing
    rate.sleep();

  }

}
