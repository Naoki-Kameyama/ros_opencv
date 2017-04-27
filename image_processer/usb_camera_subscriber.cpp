#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

class ImageConverter 
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;
	image_transport::Publisher pub;

public:
	ImageConverter()
		:it(nh)
	{
		sub=it.subscribe("/usb_cam/image_raw",1,
			&ImageConverter::imageCallback,this);
		pub=it.advertise("converter_output_image",1);
		
//		cv::namedWindow("Original_Image");
//		cv::startWindowThread();
	}

	~ImageConverter()
	{
//		cv::destroyWindow("Original_Image");
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
			cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//			cv::imshow("Image_Subscriber",cv_bridge::toCvShare(msg,
//				"bgr8")->image);
//			cv::imshow("Original_Image", cv_ptr->image);
//			cv::waitKey(3);
			
//			pub.publish(msg);
			pub.publish(cv_ptr->toImageMsg());
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
			msg->encoding.c_str());
			return ;
		}
	
	}
};
int main(int argc,char **argv)
{
	ros::init(argc,argv,"usb_camera_subscriber");
	ImageConverter ic;
	ros::spin();
	return 0;
}

