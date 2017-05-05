// http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
// scan_cloud.cpp
// TODO: change topic name
// sudo apt-get install python-catkin-build-tools
// occupancy grid utils github
// at 0.7m 30 is size of post in ranges indices at 20m 1 indeces is the size
// tmux linux terminal
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>
#include <vector>

using namespace std;

vector<int> detectPost(  const vector<float> );
bool inError( float, float, float );


class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan", 10),
    laser_notifier_(laser_sub_,listener_, "map", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "map",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // convert range index to position
    vector<int> posts = detectPost( scan_in->ranges );
    vector<geometry_msgs::Point32> postpos;
    for( unsigned int n = 0; n < posts.size(); n++ )
    {
        postpos.push_back( cloud.points[posts[n]] );
        // ROS_INFO_STREAM( cloud.points[posts[n]] );
	}
    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "scan_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;

}


const float MAX_RANGE = 4.0;
const float MIN_RANGE = 0.5;
const int MIN_WIDTH = 5;
const int MAX_WIDTH = 30;
vector<int> detectPost( const vector<float> ranges ) {

    bool is_post = false;
    float range = 0;
    int width = 0;
    int half_width = 0;
    float min_range = 0.0;
    float prior = 0.0;
    int pindex = 0;
    vector<int> posts;
    int expected_width;
    for( unsigned int n = 0; n < 720; n++ ) {

        range = ranges[n];

        if( range < MAX_RANGE && range > MIN_RANGE ) {

            // switch( (int)range ) {
            //     case 1:
            //         expected_width = 21;
            //     break;
            //     case 2:
            //         expected_width = 10;
            //     break;
            //     case 3:
            //         expected_width = 8;
            //     break;
            //     case 4:
            //         expected_width = 6;
            //     break;
            // }

            // for( unsigned int m = n; m <= n + expected_width; m++ ) {

            //     // difference between scans is too big to be a post
            //     if( !inError( range, ranges[n], 0.0168 ) )
            //         break;

            //     // if( m - n < MIN_WIDTH )
            //         // continue; 

            //     // no post should be longer than this
            //     // if( m - n > MAX_WIDTH )
            //     //     break;
                
            //     // if( inError( range, ranges[n], 0.0168 ) )
            //     //     cout << "NEW, post detected! Range: " << ranges[n] << " width: " << m - n << endl;
            //     if( m == n + expected_width - 1 )
            //         cout << "NEW, post detected! Range: " << ranges[n] << " width: " << (n + expected_width - 1) - m << endl;
            // }


        }


        if( range > MAX_RANGE || range < MIN_RANGE ) {

            // cout << "bad range" << endl;
            prior = 0.0;
            min_range = 0.0;
            width = 0;
            half_width = 0;	
 	        pindex = 0;
            continue;

        } else if( prior > MAX_RANGE || prior < MIN_RANGE ) {

            //cout << "left range: " << range << endl;
            prior = range;
            min_range = range;
            width = 1;
            half_width = 1;		
 	        pindex = 0;

        } else if( min_range > range ) {

           // cout << "left range: " << range << endl;
            min_range = range;
            width++;
            half_width = width;
	        pindex = n;

        } else if( min_range < range ) {

            if( half_width > 0 ) {
                
               // cout << "right range: " << range << endl;
                width++;
                half_width--;

            } else {

                //cout << "right range: " << range << endl;
                width++;
                if( width < MAX_WIDTH && width > MIN_WIDTH ) {

                    // the further away the smaller the width should be
                    if( ( width > 10 && min_range < 2 ) || ( width < 10 && min_range > 2 ) ) {

                        cout << "post detected, min-range:" << min_range << " width: " << width << endl;
                        // cout << pindex << endl;
                        posts.push_back( pindex );
                
                    }
                
                }

                prior = 0.0;
                min_range = 0.0;
                width = 0;
                half_width = 0;
		        pindex = 0;

            }

        } 


    }

    return posts;
}


bool inError( float x, float y, float error ) {
    
    float min = x - (x * error);
    float max = x + (x * error);

    return y > min && y < max;

}