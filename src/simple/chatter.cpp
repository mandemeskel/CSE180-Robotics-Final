#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdlib.h>

using namespace std;

#define NODE_NAME "chatter"
#define TOPIC_NAME1 "/simple/integers"
#define TOPIC_NAME2 "/simple/strings"

int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher 1
  ros::Publisher pub_ints = nh.advertise<std_msgs::Int32>( TOPIC_NAME1, 100 );
  ros::Publisher pub_strings = nh.advertise<std_msgs::String>( TOPIC_NAME2, 100 );

  // set publishing rate in
  ros::Rate rate(10);

  // npositive numbers
  int num = 1;

  while( ros::ok ) {

    // create msgs
    std_msgs::Int32 int_msg;
    std_msgs::String string_msg;

    // set messages
    int_msg.data = num;
    string_msg.data = "Michael T. Andemeskel";

    // publish messages
    pub_ints.publish( int_msg );
    pub_strings.publish( string_msg );

    // update num
    num++;

    // throttle the publisher
    rate.sleep();

  }

}
