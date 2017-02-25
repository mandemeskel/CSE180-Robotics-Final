#include <ros/ros.h>
#include <turtlesim/Color.h>
#include <stdlib.h>

using namespace std;

#define NODE_NAME "printcolor"
#define TOPIC_NAME "/turtle1/color"

int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( TOPIC_NAME, 100 );

}
