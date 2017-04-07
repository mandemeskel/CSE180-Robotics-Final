#include <ros/ros.h>
#include <sensor_msgs/SetCameraInfo.h>

int main(int argc,char **argv) {

    ros::init(argc,argv,"serviceclient");

    ros::NodeHandle nh;
    ros::ServiceClient client =
       nh.serviceClient<sensor_msgs::SetCameraInfo>("/camera/set_camera_info");
    client.waitForExistence();
    
    sensor_msgs::SetCameraInfo service;

    service.request.camera_info.height = 480;
    service.request.camera_info.width = 640;
    
    if (client.call(service)) {
	ROS_INFO_STREAM("Service successfully called");
	ROS_INFO_STREAM("Status_message:" << service.response.status_message);
	ROS_INFO_STREAM("Success flag:" << service.response.success);
    }
    else {
	ROS_ERROR_STREAM("Error while calling service");
	ROS_ERROR_STREAM("Status_message:" << service.response.status_message);
	ROS_ERROR_STREAM("Service success:" << service.response.success);
    }

    return 0;
}
