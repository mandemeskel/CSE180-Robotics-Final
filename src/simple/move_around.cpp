#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <cmath>

using namespace std;

#define NODE_NAME "move_around"
#define PUB_TOPIC_NAME "/mobile_base/commands/velocity"
#define SUB_TOPIC_NAME "/odom"
#define SUB_TOPIC_NAME1 "/mobile_base/sensors/imu_data"


bool printed_odom = false;

void callbackOdom( const nav_msgs::Odometry & );
void callbackImu( const sensor_msgs::Imu & );


int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( PUB_TOPIC_NAME, 100 );

  // create subscriber to odom
  ros::Subscriber sub_odom = nh.subscribe( SUB_TOPIC_NAME, 1000, &callbackOdom );

  // create subscriber to imu
  ros::Subscriber sub_imu = nh.subscribe( SUB_TOPIC_NAME1, 1000, &callbackImu );

  ros::Rate rate( 1 );

  // number of messages
  int msgs_sent = 0;

  while( ros::ok() ) {

    // wait for a msg from one of our subscribed topics
    ros::spinOnce();

    // create msg to be spent
    geometry_msgs::Twist msg;

    // velocity, m/s
    msg.linear.x = 1;

    // angular velocity, rad/s
    if( msgs_sent % 2 == 0) {
      msg.angular.z = M_PI_2;
      msg.linear.x = 0;
      msgs_sent = 0;
    }

    // send msg
    pub.publish( msg );

    // sent messages
    msgs_sent++;

    // wait one sec for command to be executed
    // rate.reset();
    rate.sleep();

  }


}


// print odometry msg
void callbackOdom( const nav_msgs::Odometry & msg ) {

    // toggle between printing odom and imu msg
    if( printed_odom ) return;

    printed_odom = !printed_odom;
    ROS_ERROR_STREAM( msg );

}


// print Imu msg
void callbackImu( const sensor_msgs::Imu & msg ) {

    // toggle between printing odom and imu msg
    if( !printed_odom ) return;

    printed_odom = !printed_odom;
    ROS_ERROR_STREAM( msg );

}