#include <ros/ros.h>
#include <amcl/pf/eig3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>

using namespace std;

#define NODE_NAME "eigens"
#define TOPIC_NAME "/amcl_pose"

// callback function
void callback( const geometry_msgs::PoseWithCovarianceStamped & msg );

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
void callback( const geometry_msgs::PoseWithCovarianceStamped & msg ) {

    // extrac covariance matrix for pose estimate
    // float cov = msg.pose.covariance;
    double A1[3][3] = {
        { msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2] },
        { msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[8] },
        { msg.pose.covariance[12], msg.pose.covariance[13], msg.pose.covariance[14] }
    };

    double V1[3][3] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 }
    };

    double d1[3] = { 0, 0, 0 };

    // compute eigenvectors and eigenvalues
    // eigen_decomposition( A1, V1, d1 );



    double A2[3][3] = {
        { msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5] },
        { msg.pose.covariance[9], msg.pose.covariance[10], msg.pose.covariance[11] },
        { msg.pose.covariance[15], msg.pose.covariance[16], msg.pose.covariance[17] }
    };

    double V2[3][3] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 }
    };

    double d2[3] = { 0, 0, 0 };

    // compute eigenvectors and eigenvalues
    // eigen_decomposition( A2, V2, d2 );



    double A3[3][3] = {
        { msg.pose.covariance[18], msg.pose.covariance[19], msg.pose.covariance[20] },
        { msg.pose.covariance[24], msg.pose.covariance[25], msg.pose.covariance[26] },
        { msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[32] }
    };

    double V3[3][3] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 }
    };

    double d3[3] = { 0, 0, 0 };

    // compute eigenvectors and eigenvalues
    // eigen_decomposition( A3, V3, d3 );



    double A4[3][3] = {
        { msg.pose.covariance[21], msg.pose.covariance[22], msg.pose.covariance[23] },
        { msg.pose.covariance[27], msg.pose.covariance[28], msg.pose.covariance[29] },
        { msg.pose.covariance[33], msg.pose.covariance[34], msg.pose.covariance[35] }
    };

    double V4[3][3] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 }
    };

    double d4[3] = { 0, 0, 0 };

    // compute eigenvectors and eigenvalues
    // eigen_decomposition( A4, V4, d4 );


    // ROS_ERROR_STREAM( setprecision(2) << fixed
    //     << msg
    // );

}
