#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
	        cv_bridge::CvImagePtr cv_ptr;
		cv::imshow("Image_Subscriber",cv_bridge::toCvShare(msg,
					"rgb8")->image);
		
		if(cv::waitKey(30)>=0)
			ros::shutdown();
		}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
			msg->encoding.c_str());
	}
}

int main(int argc,char **argv)
{

	ros::init(argc,argv,"camera_subscriber");


	ros::NodeHandle nh;
	cv::namedWindow("Image_Subscriber");
	cv::startWindowThread();



	image_transport::ImageTransport it(nh);	
	image_transport::Subscriber sub;


	sub = it.subscribe("/usb_cam/image_raw",1,imageCallback);



	ros::spin();
	cv::destroyWindow("camera_subscriber_sub_after");

	return EXIT_SUCCESS;
}

