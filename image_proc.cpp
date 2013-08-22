#include "image_proc.h"
#include "constants.h"
#include "utils.h"
#include "actuator.h"
#include "sensor.h"

ImageProc gImageProc;

bool ImageProc::isParaExist(IplImage* src)
{
	if(src == NULL)
	{
		Debug::print(LOG_SUMMARY, "Para detection: Unable to get Image\r\n");
		return true;
	}
	unsigned long pixelCount = 0;
	int x = 0, y = 0;
	uchar H, S, V;
	uchar minH, minS, minV, maxH, maxS, maxV;
    
	CvPixelPosition8u pos_src;
	uchar* p_src;
	IplImage* tmp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
    
	//HSVに変換
	cvCvtColor(src, tmp, CV_RGB2HSV);
    
	CV_INIT_PIXEL_POS(pos_src, (unsigned char*) tmp->imageData,
                      tmp->widthStep,cvGetSize(tmp), x, y, tmp->origin);
    
	minH = 113;	maxH = 120;
	minS = 100;	maxS = 255;
	minV = 120;	maxV = 255;
	for(y = 0; y < tmp->height; y++) {
		for(x = 0; x < tmp->width; x++) {
			p_src = CV_MOVE_TO(pos_src, x, y, 3);
            
			H = p_src[0];
			S = p_src[1];
			V = p_src[2];
            
			if( minH <= H && H <= maxH &&
               minS <= S && S <= maxS &&
               minV <= V && V <= maxV
               ) {
				++pixelCount;//閾値範囲内のピクセル数をカウント
			}
		}
	}
	double ratio = (double)pixelCount / tmp->height / tmp->width;
	Debug::print(LOG_SUMMARY, "Para ratio: %f\r\n",ratio);
	return ratio > SEPARATING_PARA_DETECT_THRESHOLD;
}
bool ImageProc::isSky(IplImage* pImage)
{
	const static int PIC_SIZE_W = 320;
	const static int PIC_SIZE_H = 240;
	const static int SKY_THRESHOLD = 200;		// 閾値以上ではあれば空だと判定
	const static int DIV_NUM_HSV = 120;			// HSVのサンプリングの間隔
	const static double COVER_RATE = 0.6;		// 画像が占めている空の割合がこの値以上だと空だと判定
	const static int SKY_DETECT_COUNT = 3;

	//Temporary Images
	IplImage* pHsvImage = cvCreateImage(cvSize(PIC_SIZE_W,PIC_SIZE_H),IPL_DEPTH_8U, 3);//HSV(8bits*3channels)	

	//Temporary Matrixes (for mixChannels)
	cv::Mat hsv_mat = cv::cvarrToMat(pHsvImage);
	
	//BGR->HSV
	cvCvtColor(pImage,pHsvImage,CV_BGR2HSV);

	int newValue_h = 0, newValue_s = 0, newValue_v = 0, newValue_sum = 0, sky = 0;
	int detect_count = 0;

	for(int i = 0;i < PIC_SIZE_H - 1;++i)
	{
		for(int j = 0; j < DIV_NUM_HSV;++j){
			int div_width = PIC_SIZE_W / DIV_NUM_HSV;
			newValue_h += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * i + j * div_width * 3];        // H
			newValue_s += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * i + j * div_width* 3 + 1];    // S
			newValue_v += (unsigned char)pHsvImage->imageData[pHsvImage->widthStep * i + j * div_width* 3 + 2];    // V
		}

		newValue_h /= DIV_NUM_HSV;
		newValue_s /= DIV_NUM_HSV;
		newValue_v /= DIV_NUM_HSV;
	
		if(newValue_h < 80 || newValue_h > 320){
			newValue_h = 0;
		}

		newValue_sum = newValue_h + newValue_v;

		if(newValue_sum > SKY_THRESHOLD){
			detect_count++;
			if(detect_count > SKY_DETECT_COUNT){
				sky = i;
			}
		}
		else{
			detect_count = 0;
		}

		newValue_h = 0;
		newValue_s = 0;
		newValue_v = 0;
		newValue_sum = 0;
	}
	
	bool isSky = sky > PIC_SIZE_H * COVER_RATE;
	Debug::print(LOG_SUMMARY, "%d / %f (%s)\r\n", sky, PIC_SIZE_H * COVER_RATE,isSky ? "found sky" : "sky not found");
	

	//Release resources
	cvReleaseImage(&pHsvImage);

	return isSky;
}
bool ImageProc::isWadachiExist(IplImage* pImage)
{
	const static int DIV_NUM = 20;
	const static int PIC_SIZE_W = 320;
	const static int PIC_SIZE_H = 240;
	const static int DELETE_H_THRESHOLD = 80;
	const static double RATE = 2.5;
	const static double PIC_CUT_RATE = 0.65;

	IplImage *src_img, *dst_img1, *tmp_img;
	double risk[DIV_NUM], risk_rate[DIV_NUM];
	CvSize pic_size = cvSize(PIC_SIZE_W, PIC_SIZE_H);

	src_img = cvCreateImage(pic_size, IPL_DEPTH_8U, 1);
	cvCvtColor(pImage, src_img, CV_BGR2GRAY);
	
	cvRectangle(src_img, cvPoint(0, 0),cvPoint(PIC_SIZE_W, PIC_SIZE_H * PIC_CUT_RATE) ,cvScalar(0), CV_FILLED, CV_AA);

	tmp_img = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_16S, 1);
	dst_img1 = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_8U, 1);
		
	// SobelフィルタX方向
	cvSobel (src_img, tmp_img, 1, 0, 3);
	cvConvertScaleAbs (tmp_img, dst_img1);

	// 2値化
	cvThreshold (dst_img1, dst_img1, DELETE_H_THRESHOLD, 255, CV_THRESH_BINARY);

	// 水平方向のエッジSum
	int height = src_img->height / DIV_NUM;
	double risk_sum = 0, risk_ave = 0;
	bool wadachi_find = false;
	for(int i = 0;i < DIV_NUM;++i)
	{
		cvSetImageROI(dst_img1, cvRect(0, height * i, src_img->width, height));//Set image part
		risk_sum += risk[i] = sum(cv::cvarrToMat(dst_img1))[0];
		cvResetImageROI(dst_img1);//Reset image part (normal)
	}

	// 平均
	risk_ave = risk_sum / DIV_NUM;

	//Draw graph
	for(int i= DIV_NUM-1; i>=0; --i){
		risk_rate[i] = risk[i] / risk_sum;

		if(i<DIV_NUM - 1){
			if(risk_rate[i] / risk_rate[i-1] > RATE && risk_rate[i] > risk_ave){
				wadachi_find = true;
			}
		}
	}

	if(wadachi_find){
		Debug::print(LOG_SUMMARY, "Wadachi Found\r\n");
		gBuzzer.start(100);
	}
	else{
		Debug::print(LOG_SUMMARY, "Wadachi Not Found\r\n");
	}

	cvReleaseImage (&src_img);
	cvReleaseImage (&dst_img1);
	cvReleaseImage (&tmp_img);

	return wadachi_find;
}
int ImageProc::wadachiExiting(IplImage* pImage)
{
	IplImage* src_img = pImage;
	const static int DIV_NUM = 5;
	IplImage *gray_img, *dst_img1, *tmp_img;
	double risk[DIV_NUM];

	if(src_img == NULL)
	{
		Debug::print(LOG_SUMMARY, "Escaping: Unable to get Image for Camera Escaping!\r\n");
		return INT_MAX;
	}
	CvSize size = cvSize(src_img->width,src_img->height);

	gray_img = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvCvtColor(src_img, gray_img, CV_BGR2GRAY);
	cvRectangle(gray_img, cvPoint(0, 0),cvPoint(src_img->width, src_img->height * 2 / 5),cvScalar(0), CV_FILLED, CV_AA);

	// Medianフィルタ
	cvSmooth (gray_img, gray_img, CV_MEDIAN, 5, 0, 0, 0);
		
	tmp_img = cvCreateImage(size, IPL_DEPTH_16S, 1);
	dst_img1 = cvCreateImage(size, IPL_DEPTH_8U, 1);
		
	// SobelフィルタX方向
	cvSobel(gray_img, tmp_img, 1, 0, 3);
	cvConvertScaleAbs (tmp_img, dst_img1);
	cvThreshold (dst_img1, dst_img1, 50, 255, CV_THRESH_BINARY);

	//Sum
	int width = src_img->width / DIV_NUM;
	double risksum = 0;
	int i;

    
	for(i = 0;i < DIV_NUM;++i)
	{
		cvSetImageROI(dst_img1, cvRect(width * i,0,width,src_img->height));//Set image part
		risksum += risk[i] = sum(cv::cvarrToMat(dst_img1))[0];
		cvResetImageROI(dst_img1);//Reset image part (normal)
	}

	//Draw graph
	for(i = 0;i < DIV_NUM;++i){
		cvRectangle(dst_img1, cvPoint(width * i,src_img->height - risk[i] / risksum * src_img->height),cvPoint(width * (i + 1),src_img->height),cvScalar(255), 2, CV_AA);
	}

    
	int min_id = 0;
    int shikiiMin = 70000;
    int shikiiMax = 150000;
    int shikiiMinCount = 0;
    int shikiiMaxCount = 0;
    
    for(int i=0; i<DIV_NUM; ++i){
        if(risk[i] < shikiiMin)
            shikiiMinCount++;
        if(risk[i] > shikiiMax)
            shikiiMaxCount++;
    }
    
    if(shikiiMinCount >= 3){
        min_id = 5;
    }else if(shikiiMaxCount >= 3){
        min_id = (risk[0] > risk[DIV_NUM - 1]) ? DIV_NUM - 1 : 0;
    }else{
        for(int i=1; i<DIV_NUM; ++i){
            if(risk[min_id] > risk[i]){
                min_id = i;
            }
        }
    }
    
    for(i=0; i<DIV_NUM; i++){
        Debug::print(LOG_SUMMARY, " area %d : %f\n" ,i,risk[i]);
    }
    
    Debug::print(LOG_SUMMARY, " min id : %d\n",min_id);
    
	cvReleaseImage (&dst_img1);
	cvReleaseImage (&tmp_img);

	switch(min_id){
		case 0:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Left\r\n");
			return -1;
		case DIV_NUM - 1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Right\r\n");
			return 1;
		default:
            Debug::print(LOG_SUMMARY, "Wadachi kaihi:Go Straight\r\n");
			return 0;
	}
}
bool ImageProc::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("predict") == 0)
		{
			isWadachiExist(gCameraCapture.getFrame());
			return true;
		}else if(args[1].compare("exit") == 0)
		{
			wadachiExiting(gCameraCapture.getFrame());
			return true;
		}else if(args[1].compare("sky") == 0)
		{
			isSky(gCameraCapture.getFrame());
			return true;
		}else if(args[1].compare("para") == 0)
		{
			isParaExist(gCameraCapture.getFrame());
			return true;
		}
		return false;
	}
	Debug::print(LOG_SUMMARY, "image [predict/exit/sky/para]  : test program\r\n");
	return true;
}
ImageProc::ImageProc()
{
	setName("image");
	setPriority(UINT_MAX,UINT_MAX);
}
ImageProc::~ImageProc()
{
}
