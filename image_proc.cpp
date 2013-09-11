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
	cvCvtColor(src, tmp, CV_BGR2HSV);
    
	CV_INIT_PIXEL_POS(pos_src, (unsigned char*) tmp->imageData,
                      tmp->widthStep,cvGetSize(tmp), x, y, tmp->origin);
    
	//orange para
	minH = 0;	maxH = 33;
	minS = 170;	maxS = 255;
	minV = 100;	maxV = 255;

	//yellow para
	/*minH = 20;	maxH = 33;
	minS = 70;	maxS = 255;
	minV = 100;	maxV = 255;*/

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
	cvReleaseImage(&tmp);
	Debug::print(LOG_SUMMARY, "Para ratio: %f\r\n",ratio);
	return ratio > SEPARATING_PARA_DETECT_THRESHOLD;
}
bool ImageProc::isSky(IplImage* src)
{
	const static double SKY_DETECT_THRESHOLD = 0.9;
	if(src == NULL)
	{
		Debug::print(LOG_SUMMARY, "Sky detection: Unable to get Image\r\n");
		return false;
	}
	unsigned long pixelCount = 0;
	int x = 0, y = 0;
	uchar H, S, V;
	uchar minH, minS, minV, maxH, maxS, maxV;
    
	CvPixelPosition8u pos_src;
	uchar* p_src;
	IplImage* tmp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
    
	//HSVに変換
	cvCvtColor(src, tmp, CV_BGR2HSV);
    
	CV_INIT_PIXEL_POS(pos_src, (unsigned char*) tmp->imageData,
                      tmp->widthStep,cvGetSize(tmp), x, y, tmp->origin);
    
	//閾値
	minH = 100;	maxH = 145;
	minS = 0;	maxS = 255;
	minV = 50;	maxV = 255;

	for(y = 0; y < tmp->height; y++) {
		for(x = 0; x < tmp->width; x++) {
			p_src = CV_MOVE_TO(pos_src, x, y, 3);
            
			H = p_src[0];
			S = p_src[1];
			V = p_src[2];
            
			if( (minH <= H && H <= maxH &&
               minS <= S && S <= maxS &&
               minV <= V && V <= maxV)
               || (H == 0 && V > 200)
               ) {
				++pixelCount;//閾値範囲内のピクセル数をカウント
			}
		}
	}

	double ratio = (double)pixelCount / tmp->height / tmp->width;
	cvReleaseImage(&tmp);
	Debug::print(LOG_SUMMARY, "Sky ratio: %f\r\n",ratio);
	return ratio >= SKY_DETECT_THRESHOLD;
}
bool ImageProc::isWadachiExist(IplImage* pImage)
{
	if(pImage == NULL)
	{
		Debug::print(LOG_SUMMARY, "Wadachi predicting: Unable to get Image\r\n");
		return false;
	}

	Debug::print(LOG_SUMMARY, "Start\n");

	const static int DIV_VER_NUM = 15;
	const static int DIV_HOR_NUM = 3;
	const static int PIC_SIZE_W = 320;
	const static int PIC_SIZE_H = 240;
	const static int DELETE_H_THRESHOLD = 50;
	const static double RATE = 2;
	const static double RISK_AVE_RATE = 0.7;
	const static int THRESHOLD = 4;
	const static int MAX_THRESHOLD = 50;
	IplImage *src_img, *dst_img, *tmp_img;
	CvSize size = cvSize(PIC_SIZE_W, PIC_SIZE_H);

	src_img = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvCvtColor(pImage, src_img, CV_RGB2GRAY);
	tmp_img = cvCreateImage (size, IPL_DEPTH_16S, 1);
	dst_img = cvCreateImage(size, IPL_DEPTH_8U, 1);
		
	// SobelフィルタX方向
	cvSobel (src_img, tmp_img, 1, 0, 3);
	cvConvertScaleAbs (tmp_img, dst_img);
	
	// 2値化
	cvThreshold (dst_img, dst_img, DELETE_H_THRESHOLD, 255, CV_THRESH_BINARY);
		
	//空カット
	CvPoint pt[(DIV_HOR_NUM+1)*2+1];
	cutSky(pImage, dst_img, pt);
	
	int risk_point[DIV_HOR_NUM]; // リスクピークのブロック番号
	int div_width = size.width / DIV_HOR_NUM;
	double risk[DIV_VER_NUM][DIV_HOR_NUM];
	double risk_rate[DIV_VER_NUM];
	int div_height  = size.height  / DIV_VER_NUM;
	double risk_sum;
	double risk_ave = 0;
	int peak_count = 0;
	
	Debug::print(LOG_SUMMARY, "  [0]    [1]    [2]  \n");

	for(int j=0; j<DIV_HOR_NUM; ++j){
		risk_sum = 0;
		for(int i=0; i<DIV_VER_NUM; ++i){
			// set image part
			cvSetImageROI(dst_img, cvRect(div_width*j, div_height*i, div_width, div_height));
			risk_sum += risk[i][j] = sum(cv::cvarrToMat(dst_img))[0];
			// reset image part (normal)
			cvResetImageROI(dst_img);
			//cvRectangle(dst_img, cvPoint(div_width*i, div_height*j), cvPoint(risk[i]/risk_sum*size.width, div_height*(j+1)), cvScalar(255), 2, CV_AA);
		}
		
		risk_ave = risk_sum / DIV_VER_NUM;
		for(int i=0; i<DIV_VER_NUM; ++i){
			risk_rate[i] = risk[i][j] / risk_sum;
		}

		Debug::print(LOG_SUMMARY, "%6.0f ", risk_ave * RISK_AVE_RATE);

		risk_point[j] = -1;
		/*
		for(int i=DIV_VER_NUM-1; i>0; --i){
			if(risk_rate[i] == 0) risk_rate[i] = 0.000001;
			if(risk_rate[i-1]/risk_rate[i] > RATE && risk[i][j] > risk_ave * RISK_AVE_RATE){
				risk_point[j] = i-1;
				peak_count++;
			}
		}
		*/
		for(int i=0; i<DIV_VER_NUM-1; ++i){
			if(risk_rate[i] == 0) risk_rate[i] = 0.000001;
			if(risk_rate[i+1]/risk_rate[i] > RATE && risk_rate[i + 1] / risk_rate[i] < MAX_THRESHOLD){
				risk_point[j] = i+1;
				peak_count++;
				break;
			}
		}
	}
	Debug::print(LOG_SUMMARY, "\n--------------------\n");
	
	for(int i=0; i<DIV_HOR_NUM; ++i){

		Debug::print(LOG_SUMMARY, " [%3d] ", risk_point[i]);
	}
	Debug::print(LOG_SUMMARY, "\n--------------------\n");

	for(int i=0; i<DIV_VER_NUM; ++i){
		Debug::print(LOG_SUMMARY, "%6.0f %6.0f %6.0f\n", risk[i][0], risk[i][1], risk[i][2]);
	}

	cvReleaseImage (&src_img);
	cvReleaseImage (&dst_img);
	cvReleaseImage (&tmp_img);

	//取得したピークの高さから行動を決定
	//0:直進，1:回避

	//一個以下だよ
	if(peak_count <= 1){
		Debug::print(LOG_SUMMARY, "Not Found Wadachi\n");
		return false;
	}
	//二個見つかったよ
	else if(peak_count == 2){
		Debug::print(LOG_SUMMARY, "Find 2 Found Peak\n");
		if(risk_point[0] == -1){
			if(risk_point[2] - risk_point[1] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Right Down -> Go Strainght\n");
				return false;
			}else if(risk_point[1] - risk_point[2] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Left Down -> Go Strainght\n");
				return false;
			}else{
				Debug::print(LOG_SUMMARY, "Horizontal -> Escape!!\n");
				return true;
			}
		}else if(risk_point[1] == -1){
			if(risk_point[2] - risk_point[0] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Right Down -> Go Strainght\n");
				return false;
			}else if(risk_point[0] - risk_point[2] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Left Down -> Go Strainght\n");
				return false;
			}else{
				Debug::print(LOG_SUMMARY, "Horizontal -> Escape!!\n");
				return true;
			}
		}else{
			if(risk_point[1] - risk_point[0] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Right Down -> Go Strainght\n");
				return false;
			}else if(risk_point[0] - risk_point[1] > THRESHOLD){
				Debug::print(LOG_SUMMARY, "Left Down -> Go Strainght\n");
				return false;
			}else{
				Debug::print(LOG_SUMMARY, "Horizontal -> Escape!!\n");
				return true;
			}
		}
	//三個もあるよ
	}else{
		Debug::print(LOG_SUMMARY, "Found 3 Peak\n");
		if(risk_point[2] - risk_point[0] > THRESHOLD){
			Debug::print(LOG_SUMMARY, "Right Down -> Go Strainght\n");
			return false;
		}else if(risk_point[0] - risk_point[2] > THRESHOLD){
			Debug::print(LOG_SUMMARY, "Left Down -> Go Strainght\n");
			return false;
		}else{
			Debug::print(LOG_SUMMARY, "Horizontal -> Escape!!\n");
			return true;
		}
	}
}
int ImageProc::wadachiExiting(IplImage* pImage)
{
	const static int DIV_HOR_NUM = 5;			// 判定に用いる列数
	const static int MEDIAN = 5;				// メディアンフィルタ用
	const static int DELETE_H_THRESHOLD = 50;	// 二値化用
	const static int THRESHOLD_COUNT = 3;		// ノイズ少のブロック数の下限
	const static double THRESHOLD_MIN = 700000;	// ノイズ数の下限

	if(pImage == NULL)
	{
		Debug::print(LOG_SUMMARY, "Escaping: Unable to get Image for Camera Escaping!\r\n");
		return (rand() % 2 != 0) ? 1 : -1;//ランダムで進行方向を決定
	}

	CvSize size = cvSize(pImage->width,pImage->height);

	IplImage *pResized = cvCreateImage(size, IPL_DEPTH_8U, 3);
	cvResize(pImage, pResized, CV_INTER_LINEAR);

	// Median Filter
	IplImage *pMedian = cvCreateImage(size,IPL_DEPTH_8U, 3);
	cvSmooth (pResized, pMedian, CV_MEDIAN, MEDIAN, 0, 0, 0);
	// Sobel Filter
	IplImage *pGray = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvCvtColor(pMedian, pGray, CV_RGB2GRAY);
	IplImage *pSobel = cvCreateImage(size, IPL_DEPTH_16S, 1);
	cvSobel(pGray, pSobel, 1, 0, 3);
	// binarization
	IplImage *pBin = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvConvertScaleAbs(pSobel, pBin);
	cvThreshold(pBin, pBin, DELETE_H_THRESHOLD, 255, CV_THRESH_BINARY);

	CvPoint pt[(DIV_HOR_NUM+1)*2+1];
	cutSky(pResized,pBin,pt);
	
	int maxH = 0, minH = size.height;
	for(int i=0; i<DIV_HOR_NUM; ++i){
		if(pt[i].y > maxH)
			maxH = pt[i].y;
		if(pt[i].y < minH)
			minH = pt[i].y;
	}
	if(maxH - minH > 150){
		return 0; // 左右どちらかに30度くらい回す
	}
	
	//Sum
	double risk[DIV_HOR_NUM], new_risk[DIV_HOR_NUM];
	double risk_sum = 0;
	int div_width  = size.width  / DIV_HOR_NUM;
	for(int i=0; i<DIV_HOR_NUM; ++i){
		// set image part
		cvSetImageROI(pBin, cvRect(div_width * i, 0, div_width, size.height));
		risk_sum += risk[i] = sum(cv::cvarrToMat(pBin))[0];
		// reset image part (normal)
		cvResetImageROI(pBin);
	}
    
	// calculate 5 heights
	double heights[DIV_HOR_NUM];
	for(int i=0; i<DIV_HOR_NUM; ++i){
		heights[i] = size.height - (pt[2*i+1].y + pt[2*(i+1)+1].y) / 2;
	}
	
	// normalize risk
	double ave_heights = 0;
	for(int i=0; i<DIV_HOR_NUM; ++i){
		ave_heights += heights[i];
	}
	ave_heights /= 5;
	for(int i=0; i<DIV_HOR_NUM; ++i){
		new_risk[i] = risk[i] * ave_heights / heights[i];
	}	
	for(int i=0; i<DIV_HOR_NUM; ++i){
		Debug::print(LOG_SUMMARY, "[%d] risk %.0f : height %.5f -> risk %.0f\r\n", i, risk[i], heights[i], new_risk[i]);
	}
	
	int minNum = 0;
	for(int i=1; i<DIV_HOR_NUM; ++i){
		if(new_risk[i] < new_risk[minNum]){
			minNum = i;
		}
	}
    
	cvReleaseImage(&pResized);
	cvReleaseImage(&pGray);
	cvReleaseImage(&pSobel);
	cvReleaseImage(&pBin);

	if(new_risk[0] < new_risk[DIV_HOR_NUM-1]){
		Debug::print (LOG_SUMMARY, "Turn left\r\n");
		return -1;
	}
	else{
		Debug::print(LOG_SUMMARY, "Turn right\r\n");
		return 1;
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
void ImageProc::cutSky(IplImage* pSrc,IplImage* pDest, CvPoint* pt)
{
	const static int DIV_VER_NUM = 80;                 // 縦に読むピクセル数
	const static int DIV_HOR_NUM = 5;                   // 判定に用いる列数
	const static int FIND_FLAG = 5;                     // 空の開始判定基準長
	const static int DELETE_H_THRESHOLD_LOW = 150;		// 空のH（色相）の範囲下限
	const static int DELETE_H_THRESHOLD_HIGH = 270;		// 空のH（色相）の範囲上限
	const static int DELETE_V_THRESHOLD_LOW = 100;      // 空のV（色相）の範囲下限
	const static int DELETE_V_THRESHOLD_HIGH = 200;     // 空のV（色相）の範囲上限
	const static int SHADOW_THRESHOLD_BOTTOM = 5;      // 下から影っぽいピクセル数
	CvSize size = cvSize(pSrc->width,pSrc->height);
	int div_width  = size.width  / DIV_HOR_NUM;
	int div_height = size.height / DIV_VER_NUM;
	
	//BGR->HSV
	IplImage* pHsv = cvCreateImage(size,IPL_DEPTH_8U, 3); //HSV(8bits*3channels)
	cvCvtColor(pSrc, pHsv,CV_BGR2HSV);
	
	bool flag;                       // 空フラグ
	pt[0] = cvPoint(0,0);            //左上端の座標を格納
	int shadow_count = 0; // 影っぽいピクセル数のカウント用

	for(int i = 0; i <= DIV_HOR_NUM; ++i)
	{
		int find_count = 0;	// 空ピクセルの数のカウント用

		// 空が判定されなかった時は上端の座標を格納
		pt[2*i+1] = cvPoint(i*div_width, 0);
		pt[2*i+2] = cvPoint((i+1)*div_width, 0);

		for(int j = DIV_VER_NUM-1; j >= 0; --j){
			int x = i * div_width;  // 取得位置のx座標
			int y = j * div_height; // 取得位置のy座標
			if(x == 0) x = 1;       // 画像左端を正常に処理するため
			
			// H値＆V値取得
			int value_h = (unsigned char)pHsv->imageData[pHsv->widthStep * y + (x - 1) * 3    ] * 2;   // H
			int value_v = (unsigned char)pHsv->imageData[pHsv->widthStep * y + (x - 1) * 3 + 2];     // V
			
			// 空判定
			flag = true;
			if(value_h < DELETE_H_THRESHOLD_LOW || DELETE_H_THRESHOLD_HIGH < value_h) //しきい値外
				flag = false;
			if(value_v < DELETE_V_THRESHOLD_LOW) //暗い時
				flag = false;
			if(value_h == 0 && value_v > DELETE_V_THRESHOLD_HIGH) //白くて明るい
				flag = true;

			//printf("h=%3d v=%3d %d \n", value_h, value_v, flag);

			if(flag){ //空ゾーン判定後、pt配列に座標を格納
				if(y > size.height - SHADOW_THRESHOLD_BOTTOM){
					shadow_count++;
					//printf("shadow\n");
					pt[2*i+1] = cvPoint(x, size.height); // 空の開始座標
					pt[2*i+2] = cvPoint((i+1)*div_width, 0);        // 次に処理する列の上端座標
					break;
				}
				find_count++;
				if(find_count > FIND_FLAG){
					pt[2*i+1] = cvPoint(x, y+FIND_FLAG*div_height); // 空の開始座標
					pt[2*i+2] = cvPoint((i+1)*div_width, 0);        // 次に処理する列の上端座標
					break;
				}
			}
			else{
				find_count = 0; // 空でないためカウント数リセット
			}
		}
		//printf("pt[%d]=(%2d, %2d)\n", 2*i, pt[2*i].x, pt[2*i].y);
		//printf("pt[%d]=(%2d, %2d)\n", 2*i+1, pt[2*i+1].x, pt[2*i+1].y);
	}
	if(shadow_count >= (DIV_HOR_NUM+1)/2){
		shadow_count = 0;
		for(int i = 0; i <= DIV_HOR_NUM; ++i)
		{
			int find_count = 0;	// 轍ピクセルの数のカウント用
			
			// 轍が判定されなかった時は下端の座標を格納
			pt[2*i+1] = cvPoint(i*div_width, size.height);
			pt[2*i+2] = cvPoint((i+1)*div_width, 0);

			for(int j = 0; j < DIV_VER_NUM; ++j){
				int x = i * div_width;  // 取得位置のx座標
				int y = j * div_height; // 取得位置のy座標
				if(x == 0) x = 1;       // 画像左端を正常に処理するため
			
				// H値＆V値取得
				int value_h = (unsigned char)pHsv->imageData[pHsv->widthStep * y + (x - 1) * 3    ] * 2;   // H
				int value_v = (unsigned char)pHsv->imageData[pHsv->widthStep * y + (x - 1) * 3 + 2];     // V
			
				// 轍判定（空の時true）
				flag = true;
				if(value_h < DELETE_H_THRESHOLD_LOW || DELETE_H_THRESHOLD_HIGH < value_h) //しきい値外
					flag = false;
				if(value_v < DELETE_V_THRESHOLD_LOW) //暗い時
					flag = false;
				if(value_h == 0 && value_v > DELETE_V_THRESHOLD_HIGH) //白くて明るい
					flag = true;
				//printf("h=%3d v=%3d %d \n", value_h, value_v, flag);

				if(!flag){ //轍ゾーン判定後、pt配列に座標を格納
					find_count++;
					if(find_count > FIND_FLAG){
						shadow_count++;
						printf("no shadow\n");
						pt[2*i+1] = cvPoint(x, y-FIND_FLAG*div_height); // 轍の開始座標
						pt[2*i+2] = cvPoint((i+1)*div_width, 0);        // 次に処理する列の上端座標
						break;
					}
				}
				else{
					find_count = 0; // 空でないためカウント数リセット
				}
			}
			//printf("pt[%d]=(%2d, %2d)\n", 2*i, pt[2*i].x, pt[2*i].y);
			//printf("pt[%d]=(%2d, %2d)\n", 2*i+1, pt[2*i+1].x, pt[2*i+1].y);
		}
		if(shadow_count != DIV_HOR_NUM+1)
		{
			for(int i=0; i<=DIV_HOR_NUM; ++i){
				pt[2*i+1] = cvPoint(i*div_width, 0); // 轍の開始座標
				pt[2*i+2] = cvPoint((i+1)*div_width, 0);        // 次に処理する列の上端座標
			}
			return;
		}
	}

	// 空カット
	int npts[1] = {4};	// 塗りつぶす図形の頂点数
	CvPoint pts[4];
	for(int i=0; i<DIV_HOR_NUM; ++i){
		pts[0] = pt[2*i];
		pts[1] = pt[2*i+1];
		pts[2] = pt[2*(i+1)+1];
		pts[3] = pt[2*(i+1)];
		CvPoint *ptss[1] = {&pts[0]};
		cvFillPoly(pDest, ptss, npts, 1, cvScalar(0), CV_AA, 0);
	}
	cvReleaseImage(&pHsv);
}

ImageProc::ImageProc()
{
	setName("image");
	setPriority(UINT_MAX,UINT_MAX);
}
ImageProc::~ImageProc()
{
}
