#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>
#include <vector>

using namespace std;

vector<int> detectPost(  const vector<float> );


class My_Filter {
    public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf2::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};


My_Filter::My_Filter() {
    
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

}

void My_Filter::scanCallback( const sensor_msgs::LaserScan::ConstPtr & scan ) {

    // sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud cloud;    
    projector_.transformLaserScanToPointCloud("map", *scan, cloud, tfListener_);

    // convert range index to position
    vector<int> posts = detectPost( scan->ranges );
    vector<geometry_msgs::Point32> postpos;
    for( unsigned int n = 0; n < posts.size(); n++ )
    {
        postpos.push_back( cloud.points[posts[n]] );
        ROS_ERROR_STREAM( cloud.points[posts[n]] );
	}

    point_cloud_publisher_.publish(cloud);

}


int main( int argc, char** argv ) {

    ros::init(argc, argv, "pointcloud");

    My_Filter filter;

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
    for( unsigned int n = 0; n < 720; n++ ) {

        range = ranges[n];

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
				cout << pindex << endl;
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

