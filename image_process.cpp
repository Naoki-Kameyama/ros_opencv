#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
 
class ImageProcesser
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;	
	image_transport::Publisher pub;
	cv::Mat org_img;						//元の画像
	cv::Mat l_img;							//左の画像
	cv::Mat r_img;							//右の画像

	cv::Mat dif_l_img;						//左の差分画像
	cv::Mat dif_r_img;						//右の差分画像
    cv::Mat dif_l_gray;                    //左の差分画像(グレースケール)
    cv::Mat dif_r_gray;                    //右の差分画像(グレースケール)

	cv::Mat prev_l_img;					//１つ前の左の画像
	cv::Mat prev_r_img;					//１つ前の右の画像
	int count;
	cv::Mat gray;
public:
	ImageProcesser()
		:it(nh)
	{	ROS_INFO_STREAM("Starrrt");
		sub=it.subscribe("converter_output_image",1,
			&ImageProcesser::ImagePreProcess,this);
   		ROS_INFO_STREAM("Starrrt");
		pub=it.advertise("processer_output",1);
		ROS_INFO_STREAM("Starrrt");
		cv::namedWindow("l_image",CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();
		cv::namedWindow("r_image",CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();	
		cv::namedWindow("dif_l_image",CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();
		cv::namedWindow("dif_r_image",CV_WINDOW_AUTOSIZE);
		cv::startWindowThread();
		count=0;
	}

	void ImagePreProcess(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	
		ROS_INFO_STREAM("ptr->mat");
		cv::Mat &im=cv_ptr->image;
		org_img=im.clone();
		//org_img=cv_ptr->image;//.clone();
		//画像分割
        ROS_INFO_STREAM("ptr->mat->Rect");			//ここでコアダンプ
		l_img=org_img(cv::Rect(0,0,1344/2,376));
//		l_img=cv_ptr->image(cv::Rect(0,0,1344/2,376));
		ROS_INFO_STREAM("ptr->mat->Rect->l");
		imshow("l_image",l_img);
		r_img=org_img(cv::Rect(1344/2,0,1344/2,376));
//		r_img=cv_ptr->image(cv::Rect(1344/2,0,1344/2,376));
		imshow("r_image",r_img);
		//ひずみ補正

		/*	
			
			未完成		
		
		*/
	
		//)
			if(count==0){
				
				count=1;
			}
			else{
				
				cv::cvtColor(org_img,gray,CV_BGR2GRAY);
				//dif_l_img = l_img - prev_l_img;
				//dif_r_img = r_img - prev_r_img;
				ROS_INFO_STREAM("StarrrtColor");
			//	imshow("dif_l_img",prev_l_img);//dif_l_img);
			//	cv::cvtColor(dif_l_img,dif_l_gray,CV_BGR2GRAY);
			//	cv::cvtColor(dif_r_img,dif_r_gray,CV_BGR2GRAY);
			//	cv::cvtColor(l_img,gray,CV_BGR2GRAY);
			//	cv::cvtColor(dif_r_img,dif_r_gray,CV_BGR2GRAY);
			//	cv::cvtColor(dif_l_img,dif_l_gray,CV_BGR2GRAY);
			//	cv::cvtColor(dif_r_img,dif_r_gray,CV_BGR2GRAY);
			//	absdiff(l_img,prev_l_gray,dif_l_img);
			//	cv::threshold(dif_l_gray,dif_l_gray,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
			//	cv::threshold(dif_l_gray,dif_l_gray,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);	
				ROS_INFO_STREAM("imshow");	
				imshow("dif_l_img",gray);//prev_l_img);//dif_l_img);
				//imshow("dif_r_img",dif_r_img);
			/*

	        	未完成
	
			*/
			}
			
				
			//prev_l_img=l_img;
			//l_img.copyTo(prev_l_img);
			//prev_r_img= r_img;
			//r_img.copyTo(prev_r_img);
	}
	

};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"Image_Processer");
	ImageProcesser improi;
	ros::spin();
	return 0;
}
