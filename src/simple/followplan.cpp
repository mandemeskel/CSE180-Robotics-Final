#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

#define NODE_NAME "followplan"
#define PUB_TOPIC_NAME "/turtle1/cmd_vel"
#define SUB_TOPIC_NAME "commands"

void callback( const std_msgs::Char & );

struct Cmd {
    char cmd;
    bool execute;
} cmd;


int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( PUB_TOPIC_NAME, 100 );

  // create subscriber
  ros::Subscriber sub = nh.subscribe( SUB_TOPIC_NAME, 1000, &callback );

  // publish 1 msg per sec
  ros::Rate rate( 1 );

  // struct to help us manage commands
  cmd.execute = false;

  while( ros::ok() ) {

    // listen for user commands
    ros::spinOnce();

    // creat message to send to cmd_vel
    geometry_msgs::Twist msg;

    if( cmd.execute == true ) {

        switch( cmd.cmd ) {
            // forward
            case 'F':
                msg.linear.x = 1;
                msg.angular.z = 0;
            break;
            // backward
            case 'B':
                msg.linear.x = -1;
                msg.angular.z = 0;
            break;
            // rotate -90deg, right
            case 'R':
                msg.angular.z = -M_PI / 2;
                msg.linear.x = 0;
            break;
            // rotate 90deg, left
            case 'L':
                msg.angular.z = M_PI / 2;
                msg.linear.x = 0;
            break;
        }
    
    } else {

        msg.linear.x = 0;
        msg.angular.z = 0;

    }

    // send msg
    pub.publish( msg );
    cmd.execute = false;

    // wait one sec for command to be executed
    rate.reset();
    rate.sleep();

  }

}

void callback( const std_msgs::Char & msg ) {

    cmd.cmd = msg.data;
    cmd.execute = true;

}
