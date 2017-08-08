/*
空検知のプログラム
たてにみていくよ！
*/

#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"

const static int THRESHOLD = 200;
const static int DIV_NUM = 160;
const static int FIND_FLAG = 3;

int main6( int argc, char** argv ) {
	CvSize capSize = {240,320};

	//Get current image
	IplImage* readImage = cvLoadImage ("genchi/pic1_vert.jpg", CV_LOAD_IMAGE_COLOR);
	
	IplImage* pCaptureFrame = cvCreateImage(cvSize(240,320),IPL_DEPTH_8U, 3);
	cvResize(readImage, pCaptureFrame, CV_INTER_LINEAR);

	// Medianフィルタ
	cvSmooth (pCaptureFrame, pCaptureFrame, CV_MEDIAN, 5, 0, 0, 0);

	//Temporary Images
	IplImage* pHsvImage = cvCreateImage(capSize,IPL_DEPTH_8U, 3);//HSV(8bits*3channels)
	IplImage* pHImage = cvCreateImage(capSize,IPL_DEPTH_8U, 1);//Grayscale(kido)
	IplImage* pSImage = cvCreateImage(capSize,IPL_DEPTH_8U, 1);//Grayscale(kido)
	IplImage* pVImage = cvCreateImage(capSize,IPL_DEPTH_8U, 1);//Grayscale(kido)
	
	//Temporary Matrixes (for mixChannels)
	cv::Mat hsv_mat = cv::cvarrToMat(pHsvImage)
		,h_mat = cv::cvarrToMat(pHImage)
		,s_mat = cv::cvarrToMat(pSImage)
		,v_mat = cv::cvarrToMat(pVImage);
	
	//BGR->HSV
	cvCvtColor(pCaptureFrame,pHsvImage,CV_BGR2HSV);

	//Get kido Channel
	const int from_to_h[] = {0,0}, from_to_s[] = {1,0}, from_to_v[] = {2,0};
	cv::mixChannels(&hsv_mat, 1, &h_mat, 1, from_to_h, 1 );
	cv::mixChannels(&hsv_mat, 1, &s_mat, 1, from_to_s, 1 );
	cv::mixChannels(&hsv_mat, 1, &v_mat, 1, from_to_v, 1 );

	//Draw graph on pKidoImage & find gake
	int lastValue_h = (unsigned char)pHsvImage->imageData[pHsvImage->widthStep*(capSize.height / 2)],
		lastValue_s = (unsigned char)pHsvImage->imageData[pHsvImage->widthStep*(capSize.height / 2) + 1],
		lastValue_v = (unsigned char)pHsvImage->imageData[pHsvImage->widthStep*(capSize.height / 2) + 2],
		lastValue_sum = pCaptureFrame->imageData[pHsvImage->widthStep*(capSize.height / 2) / 2];

	int newValue_h = 0, newValue_s = 0, newValue_v = 0, newValue_sum = 0, gake = 0;
	int find_count = 0;

	for(int i = 0; i < capSize.width; ++i)
	{
		for(int j = DIV_NUM-1; j >= 0; --j){
			int div_height = capSize.height / DIV_NUM;
			newValue_h += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * j * div_height + i * 3];        // H
			newValue_s += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * j * div_height + i * 3 + 1];    // S
			newValue_v += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * j * div_height + i * 3 + 2];    // V
		}

		newValue_h /= DIV_NUM;
		newValue_s /= DIV_NUM;
		newValue_v /= DIV_NUM;
		
		if(newValue_h < 80 || newValue_h > 320){
			newValue_h = 0;
		}

		newValue_sum = newValue_h + newValue_v;

		cvLine(pHImage,cvPoint(i,lastValue_h),cvPoint(i+1,newValue_h),cvScalar(255),1,CV_AA ,0);
		cvLine(pSImage,cvPoint(i,lastValue_s),cvPoint(i+1,newValue_s),cvScalar(255),1,CV_AA ,0);
		cvLine(pVImage,cvPoint(i,lastValue_v),cvPoint(i+1,newValue_v),cvScalar(0),1,CV_AA ,0);
		cvLine(pCaptureFrame,cvPoint(i,lastValue_sum),cvPoint(i+1,newValue_sum),cvScalar(0),1,CV_AA ,0);

		lastValue_h = newValue_h;
		lastValue_s = newValue_s;
		lastValue_v = newValue_v;
		lastValue_sum = newValue_sum;

		printf("%d\n", newValue_sum);

		if(newValue_sum > THRESHOLD){
			find_count++;
			if(find_count > FIND_FLAG){
				gake = i;
			}
		}

		newValue_h = 0;
		newValue_s = 0;
		newValue_v = 0;
		newValue_sum = 0;
	}


	//Show result
	cvNamedWindow( "origin", CV_WINDOW_AUTOSIZE );
	cvShowImage( "origin", pCaptureFrame );
	cvNamedWindow( "H", CV_WINDOW_AUTOSIZE );
	cvShowImage( "H", pHImage );
	cvNamedWindow( "S", CV_WINDOW_AUTOSIZE );
	cvShowImage( "S", pSImage );
	cvNamedWindow( "V", CV_WINDOW_AUTOSIZE );
	cvShowImage( "V", pVImage );
	cvWaitKey (0);

	//Release resources
	cvReleaseImage(&readImage);
	cvReleaseImage(&pCaptureFrame);
	cvReleaseImage(&pHsvImage);
	cvReleaseImage(&pHImage);
	cvReleaseImage(&pSImage);
	cvReleaseImage(&pVImage);
	cvDestroyWindow("H");
	cvDestroyWindow("S");
	cvDestroyWindow("V");
	cvDestroyWindow("origin");
	return 0;
}
