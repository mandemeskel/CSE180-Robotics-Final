#include <ros/ros.h>
#include <turtlesim/Color.h>
#include <stdlib.h>

using namespace std;

#define NODE_NAME "printcolor"
#define TOPIC_NAME "/turtle1/color_sensor"

// callback function
void callback( const turtlesim::Color & msg );

int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create the Subscriber and set the topic to listen to
  ros::Subscriber sub = nh.subscribe( TOPIC_NAME, 1000, &callback );

  // listen to topic
  ros::spin();

}


// callback function
void callback( const turtlesim::Color & msg ) {

  // NOTE: you have to msg properties as int because they are unint_8 which ROS_INFO_STREAM
  // does not how to add to strings, it prints them as question marks
  ROS_ERROR_STREAM( setprecision(2) << fixed
    << "Color( r= " << (int)msg.r << " g= " << (int)msg.g << " b= " << (int)msg.b << " )"
  );

}
