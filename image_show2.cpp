#include<opencv2/highgui/highgui.hpp>


int main(int argc, char* argv[])
{
        cv::Mat img;
		img=cv::imread("./sample.jpeg",1);		

        cv::imshow("window", img);//画像を表示．
		cv::waitKey(0);
	cv::destroyAllWindows();
    return 0;
}

