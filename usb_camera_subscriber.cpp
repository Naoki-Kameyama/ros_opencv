#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		ROS_INFO_STREAM("Starrrrt_func");
        cv_bridge::CvImagePtr cv_ptr;
		cv_ptr=cv_bridge::toCvCopy(msg, (sensor_msgs::image_encodings::BGR8));
//		cv::imshow("Image_Subscriber",cv_bridge::toCvShare(msg,
//					"rgb8")->image);
		cv::imshow("Image_Subscriber",cv_ptr->image);
//	    cv::imwrite("Teste.png", cv_ptr->image);
		
		ROS_INFO_STREAM("Starrrrt");

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
	ROS_INFO_STREAM("Starrrrt");
	ros::init(argc,argv,"camera_subscriber");
	ROS_INFO_STREAM("Starrrrt");

	ros::NodeHandle nh;
	cv::namedWindow("Image_Subscriber");
	cv::startWindowThread();

    ROS_INFO_STREAM("Starrrrt");

	image_transport::ImageTransport it(nh);	
	image_transport::Subscriber sub;

	ROS_INFO_STREAM("Starrrrt_sub_prev");

	sub = it.subscribe("/usb_cam/image_raw",1,imageCallback);

	ROS_INFO_STREAM("Starrrrt");

	ros::spin();
	cv::destroyWindow("camera_subscriber_sub_after");
	ROS_INFO_STREAM("Errror");
	return EXIT_SUCCESS;
}

