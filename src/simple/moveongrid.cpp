#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

#define NODE_NAME "moveongrid"
#define PUB_TOPIC_NAME "/turtle1/cmd_vel"
#define TOPIC_POSE "/turtle1/pose"
#define TOPIC_CMDS "commands"

ros::Publisher pub;
void callbackPose( const turtlesim::Pose & );
void callbackCmds( const std_msgs::Char & );
geometry_msgs::Twist createTwistMsg( char );
float correctTheta( float );

struct Cmd {
    char cmd;
    float theta;
    bool execute;
    bool check_theta;
} cmd;


int main( int argc, char ** argv ) {

  // init ros
  ros::init( argc, argv, NODE_NAME );
  ros::NodeHandle nh;

  // create publisher
  pub = nh.advertise<geometry_msgs::Twist>( PUB_TOPIC_NAME, 100 );

  // create subscriber "/turtle1/pose"
  ros::Subscriber sub_pose = nh.subscribe( TOPIC_POSE, 1000, &callbackPose );

  // create subscriber "commands"
  ros::Subscriber sub_cmds = nh.subscribe( TOPIC_CMDS, 1000, &callbackCmds );

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

        float theta = correctTheta( cmd.theta );

        if( theta != 0 ) {

            msg.linear.x = 0;
            msg.angular.z = theta;

            // send msg
            pub.publish( msg );

            // wait one sec for command to be executed
            rate.reset();
            rate.sleep();

            cmd.theta += theta;

        }
        
        msg = createTwistMsg( cmd.cmd );

        // switch( cmd.cmd ) {
        //     // forward
        //     case 'U':
        //         msg.linear.x = 1;
        //         msg.angular.z = 0;
        //     break;
        //     // backward
        //     case 'D':
        //         msg.linear.x = 1;
        //         msg.angular.z = 0;
        //     break;
        //     // right
        //     case 'R':
        //         msg.linear.x = 1;
        //         msg.angular.z = 0;
        //     break;
        //     // left
        //     case 'L':
        //         msg.linear.x = 1;
        //         msg.angular.z = 0;
        //     break;
        // }
    
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

void callbackCmds( const std_msgs::Char & msg ) {

    cmd.cmd = msg.data;
    cmd.execute = true;

}

void callbackPose( const turtlesim::Pose & msg ) {

    cmd.theta = msg.theta;

}

float correctTheta( float th ) {

    // float theta = cmd.theta;
    int theta = (int) cmd.theta;
    float turn = 0;

    if( cmd.cmd == 'U' ) { 

        if( theta == 3 ) // left
            turn = -M_PI_2;
        else if( theta == 0 ) // right
            turn = M_PI_2;
        else if( theta == -1 ) // down
            turn = M_PI;

    } else if( cmd.cmd == 'D' ) {

        if( theta == 3 ) // left
            turn = M_PI_2;
        else if( theta == 0 ) // right
            turn = -M_PI_2;
        else if( theta == 1 ) // up
            turn = M_PI;

    } else if( cmd.cmd == 'L' ) { 

        if( theta == 1 ) // up
            turn = M_PI_2;
        else if( theta == -1 ) // down
            turn = -M_PI_2;
        else if( theta == 0 ) // right
            turn = M_PI;

    } else if ( cmd.cmd == 'R' ) {

        if( theta == 1 ) // up
            turn = -M_PI_2;
        else if( theta == -1 ) // down
            turn = M_PI_2;
        else if( theta == 3 ) // left
            turn = M_PI;

    }

    return turn;

}

geometry_msgs::Twist createTwistMsg( char achar ) {
    // creat message to send to cmd_vel
    geometry_msgs::Twist msg;

    switch( achar ) {
        // move to next sell
        case 'U':
        case 'D':
        case 'R':
        case 'L':
            msg.linear.x = 1;
            msg.angular.z = 0;
        break;
        default:
            msg.linear.x = 0;
            msg.angular.z = 0;
        break;
    }

    return msg;

}