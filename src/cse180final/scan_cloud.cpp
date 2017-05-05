// http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
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

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan", 10),
    laser_notifier_(laser_sub_,listener_, "map", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
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
        ROS_INFO_STREAM( cloud.points[posts[n]] );
	}

  }
};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "scan_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;

}

// percent error of the laser we used to compare expected
// radius and calcualted radius of posts we find
const float ERROR = 0.10;
// LaserScan angle increment, used in converting from ranges index to angle
const float INCREM = 0.0065540750511;
// the radius of a post
const float POST_RADIUS = 0.07;
//maximum range of post detection
const float MAX_RANGE = 5.0;
// minimum number of scans a posts within 5 meters should have
const int MIN_WIDTH = 5;
// detects posts in LaserScan:ranges and returns a list
// of detected posts indexs, the indexs are from ranges
vector<int> detectPost( const vector<float> ranges ) {

    vector<int> posts;
	vector<float> my_ranges;
    float prior_range = 0.0;
	float range = 0.0;
	float angle = 0.0;
	float radius_calc = 0.0;
	for( int n = 0; n < 720; n++ ) {

		range = ranges[n];

		if( prior_range == 0.0 && range < MAX_RANGE ) {
			
			prior_range = range;
			my_ranges.push_back( range );	
			continue;

		// } else if( (int)range == (int)prior_range ) {
        } else if( inError( range, prior_range, 0.06 ) ) {

			my_ranges.push_back( range );
			prior_range = range;

		} else if( my_ranges.size() > MIN_WIDTH ) {

			// calculate angle relative to first scan of post
			angle = abs( my_ranges.size() * INCREM ) / 2; 

			// calculate radius of the suspected post
			radius_calc = my_ranges[0] * sinf( angle );
			
			// see if this is an actual post by comparing to actual radius
			if( inError( POST_RADIUS, radius_calc, ERROR ) ) {
            
                // add ranges index of this post to posts array
                // for use in point cloud
                posts.push_back( n );
            	// cout << " range: " << my_ranges[0] << " width:" << my_ranges.size() << " angle: " << angle << " radius_calc: " << radius_calc << endl;
            
            }
            
			my_ranges.clear();
			prior_range = 0.0;

		} else {

			my_ranges.clear();
			prior_range = 0.0;

		}

	}

    // we are missing a post or found a fraud
    if( posts.size() == 1 )
        posts.clear();
    // if( posts.size() % 2 == 1 ) {

    //     for( int p = 0; p < posts.size(); p += 2 ) {

    //         if( abs( posts[p] - posts[p+1] ) < 70 )
    //             continue;
    //         else if( abs)
    //     }

    // }

    return posts;
}


bool inError( float x, float y, float error ) {
    
    float min = x - (x * error);
    float max = x + (x * error);

    return y > min && y < max;

}