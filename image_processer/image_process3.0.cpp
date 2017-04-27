#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
 
class ImageProcesser
{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;	
	image_transport::Publisher pub;

public:
/*
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
*/
	ImageProcesser();
	~ImageProcesser();
	

};

ImageProcesser::~ImageProcesser(){
}

ImageProcesser::ImageProcesser()
	:it(nh)
{	
	pub=it.advertise("processer_output",1);
	
	cv::VideoCapture cap(0);//デバイスのオープン
	if(!cap.isOpened())
		ROS_INFO_STREAM("Failed opening camera!");

	cap.set(CV_CAP_PROP_FRAME_WIDTH,1344);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,376);
	
//１つ前のフレームを格納するために使用
	cv::Mat PreLimg;
	cv::Mat PreRimg;
	cv::Mat org_img;	

		while(1){//無限ループ
//カメラフレームの取得
			//cv::Mat fram;
			cap>>org_img;
//画像の分割
			cv::Mat Limg=org_img(cv::Rect(0,0,1344/2,376));
//			cv::imshow("l_image",Limg);
//			cv::waitKey(10);			
			cv::Mat Rimg=org_img(cv::Rect(1344/2,0,1344/2,376));
//			cv::imshow("r_image",Rimg);
//			cv::waitKey(10);
			if(PreLimg.empty()||PreRimg.empty()){	//差分を取るための処理(ループ１回目のみ処理をしない)
			}										
			else{
//グレースケール化
				cv::Mat Lgray,Rgray,PreLgray,PreRgray;
				cv::cvtColor(Limg,Lgray,CV_BGR2GRAY);
				cv::cvtColor(Rimg,Rgray,CV_BGR2GRAY);
				cv::cvtColor(PreLimg,PreLgray,CV_BGR2GRAY);
				cv::cvtColor(PreRimg,PreRgray,CV_BGR2GRAY);
				
//差分をとり２値化
				cv::Mat DifLgray,DifRgray,MaskLimg,MaskRimg;
				cv::absdiff(Lgray,PreLgray,DifLgray);
				cv::absdiff(Rgray,PreRgray,DifRgray);
//差分diffのうち、閾値thを超えている部分を1、それ以外を0としてmaskに出力
    		    cv::threshold(DifLgray,MaskLimg,5,1,cv::THRESH_BINARY);
				cv::threshold(DifRgray,MaskRimg,10,1,cv::THRESH_BINARY);
//マスクmaskのうち、1(True)の部分を白(0)に、0(False)の部分を黒(255)にしてim_maskに出力
        		cv::threshold(MaskLimg,MaskLimg,0,255,cv::THRESH_BINARY);				
				cv::threshold(MaskRimg,MaskRimg,0,255,cv::THRESH_BINARY);
//メディアンフィルタ
				cv::medianBlur(MaskLimg,MaskLimg,5);
				cv::medianBlur(MaskRimg,MaskRimg,5);
				cv::imshow("l_img",Limg);
//				cv::waitKey(10);
				cv::imshow("r_img",Rimg);
//				cv::waitKey(10);
				cv::imshow("mask_l_img",MaskLimg);
//				cv::waitKey(10);
				cv::imshow("mask_r_img",MaskRimg);
				cv::waitKey(10);
			
//輪郭抽出(左カメラ)
/*
				std::vector<cv::Mat>  contours;
        		cv::findContours(MaskLimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        
//面積が最大になるエリアを特定
				double max_area=0;
				int max_area_contour=-1;
				for(int j=0;j<contours.size();j++){
					double area=cv::contourArea(contours.at(j));
					if(max_area<area){
						max_area=area;
                		max_area_contour=j;
					}
				}
//重心算出
				int count=contours.at(max_area_contour).size();
				double x=0;
				double y=0;
				for(int k=0;k<count;k++){
					x+=contours.at(max_area_contour).at(k).x;
					y+=contours.at(max_area_contour).at(k).y;
				}
				x/=count;
				y/=count;
				cv::imshow("l_img",Limg);
				cv::imshow("l_img",Rimg);
				cv::imshow("mask_l_img",MaskLimg);
				cv::imshow("mask_r_img",MaskRimg);
				cv::waitKey(15);
*/			}//else文終了

//１つ前の画像を格納
			Limg.copyTo(PreLimg);
			Rimg.copyTo(PreRimg);

		}
		
}

	
int main(int argc,char **argv)
{
	ros::init(argc,argv,"Image_Processer");//ノード名
	ImageProcesser ImageProcesser;
	ros::spin();
	return 0;
}
