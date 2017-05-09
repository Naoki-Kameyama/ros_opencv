#include"ros/ros.h"
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>//オプティカルフロー用
#include <opencv2/features2d/features2d.hpp>
 
class ImageProcesser
{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;	
	image_transport::Publisher pub;

public:
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
//カメラ設定
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1344);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,376);
//
//	Ptr<DenseOpticalFlowExt>
	
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
			cv::Mat Limg_view=Limg.clone();//imshow用のMat
			
			cv::Mat Rimg=org_img(cv::Rect(1344/2,0,1344/2,376));

//			Limg.convertTo(Limg,CV_8UC1);
//			Rimg.convertTo(Rimg,CV_8UC1);

			

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
    		    cv::threshold(DifLgray,MaskLimg,10,1,cv::THRESH_BINARY);
				cv::threshold(DifRgray,MaskRimg,10,1,cv::THRESH_BINARY);
//マスクmaskのうち、1(True)の部分を白(0)に、0(False)の部分を黒(255)にしてim_maskに出力
        		cv::threshold(MaskLimg,MaskLimg,0,255,cv::THRESH_BINARY);				
				cv::threshold(MaskRimg,MaskRimg,0,255,cv::THRESH_BINARY);
//メディアンフィルタ
				cv::medianBlur(MaskLimg,MaskLimg,5);
				cv::medianBlur(MaskRimg,MaskRimg,5);
			
//膨張縮小処理
				cv::Mat MaskLimg_prev=MaskLimg.clone();	
				//改善の余地あり
				cv::erode(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 2);
				cv::dilate(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 2);
				cv::dilate(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 4);
				cv::erode(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 2);
//輪郭抽出(左カメラ)
				cv::Mat act;
				double sum_area=0;
				act=MaskLimg.clone();
				std::vector<std::vector<cv::Point> >  contours;
        		cv::findContours(act/*MaskLimg*/, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

				std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
				std::vector<cv::Rect> boundRect( contours.size() );

//差分画像を矩形で囲む(左上、右下の座標を取る)
				int rect_num=0;//矩形の数
				for( int i = 0; i < contours.size(); i++ )
				{
					//表示数制限
					if(contours.size()>20)
						break;
					//
					double area=cv::contourArea(contours.at(i));
					sum_area+=area;
					if(area>600){//面積が一定以上であれば//改善の余地あり
					cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true );
					boundRect[rect_num++] = cv::boundingRect(cv::Mat(contours_poly[i]) );//矩形の作成、格納
					
					}
				}

				if(sum_area<(1344*376/20.0))//改善の余地あり
				{	//カメラ移動時のその場しのぎの処理
					cv::Mat opt_PreLimg[rect_num];
					cv::Mat opt_Limg[rect_num]; 
					std::vector<cv::Point2f> points[rect_num];	//特徴点
					std::vector<cv::Point2f> newpoints[rect_num];	//移動後の特徴点
					std::vector<uchar/*unsigned*/> status[rect_num];	//オプティカルフロー用
					std::vector<float> errors;			//
//					cv::Mat errors;
					//cv::Point2f AOM;					//Amount of movementi
					std::vector<cv::KeyPoint> keypoints;
					
					for(int i=0;i<rect_num;i++)
					{
						cv::rectangle(Limg_view,boundRect[i].tl(),boundRect[i].br(),cv::Scalar(200,0,0),4,4);//動体を矩形で囲む
						//img→ grayに書き換え
						opt_Limg[i]=Lgray(cv::Rect(boundRect[i].tl(),boundRect[i].br()));//現フレームの動体と予測された範囲を矩形として取り出す
						opt_PreLimg[i]=PreLgray(cv::Rect(boundRect[i].tl(),boundRect[i].br()));//前フレームの、、以下略
					//}
//オプティカルフローの処理(取得)
//参照URL:http://opencv.jp/opencv-2svn/cpp/motion_analysis_and_object_tracking.html#cv-calcopticalflowpyrlk
//						cv::goodFeaturesToTrack(opt_PreLimg[i], points[i], 20, 0.05, 5.0);
//						cv::goodFeaturesToTrack(opt_Limg[i],newpoints[i], 20, 0.05, 5.0);

 
						cv::GoodFeaturesToTrackDetector detector(100, 0.05, 3);
						detector.detect(opt_PreLimg[i], keypoints);

						for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin(); itk != keypoints.end(); ++itk)
						{
							points[i].push_back(itk->pt);
						}
//						cv::cvtColor(opt_PreLimg[i],opt_PreLimg[i],CV_RGB2GRAY);
//						cv::cvtColor(opt_Limg[i],opt_Limg[i],CV_RGB2GRAY);

						opt_PreLimg[i].convertTo(opt_PreLimg[i],CV_8UC1);
						opt_Limg[i].convertTo(opt_Limg[i],CV_8UC1);
//						cout<<CV_8UC1<<;
//						ROS_INFO_STREAM("channels"<<opt_Limg[i].channels());
//						ROS_INFO_STREAM("channels"<<opt_PreLimg[i].channels());

//						ROS_INFO_STREAM("channels"<<opt_Limg[i].size().width<<","<<opt_Limg[i].size().height);
						ROS_INFO_STREAM("Pre:channels"<<opt_PreLimg[i].size().width<<","<<opt_Limg[i].size().height);
						cv::calcOpticalFlowPyrLK(opt_PreLimg[i],opt_Limg[i], points[i], newpoints[i], status[i], errors, cv::Size(21,21), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.05), 0);

						
						
						ROS_INFO_STREAM("Point:("<<points[i]);
/*<<","<<points[i].y<<")");*/
				//		ROS_INFO_STREAM("newPoint:("<<newpoints[i].x<<","<<newpoints[i].y<<")");

//
//						if(status==1){
							
							
//						}
//オプティカルフローの移動量x,yの計算						
										
					}
				}
				
				
					
//				cv::imshow("l_img",Limg);
				cv::imshow("l_img_rect",Limg_view);//表示用
//				cv::imshow("l_img",Rimg);
				cv::imshow("mask_l_img",MaskLimg);
//				cv::imshow("mask_l_prev",MaskLimg_prev);//変更前
//				cv::imshow("mask_r_img",MaskRimg);
				cv::waitKey(20);
			}//else文終了

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
