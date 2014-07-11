#include "image_proc.h"
#include "constants.h"
#include "utils.h"
#include "actuator.h"
#include "sensor.h"

ImageProc gImageProc;

/* ここから　2014年6月オープンラボ前に実装 */
// 実行速度：0.22~0.24sec
int ImageProc::howColorGap(IplImage* src)
{
	int x_gap = 0;												//返り値(中心からのX位置のずれ)
	cv::Mat input_img;											//カメラ
	cv::Mat hsv_img;											//HSV
	cv::Mat smooth_img;											//平滑化後
	cv::Mat hsv_color_img = cv::Mat(cv::Size(320,240),CV_8UC3);	//抽出後
	cv::Mat gray_img;											//グレースケール化後
	cv::Mat mono_img;											//二値化後
	cv::Mat tmp_img;											//一時Mat
	cv::Moments moments;										//重心計算用

	input_img = cv::cvarrToMat(src);						//カメラのストリーミング先を設定
	cv::medianBlur(input_img,smooth_img,7);					//ノイズがあるので平滑化
	cv::cvtColor(smooth_img,hsv_img,CV_BGR2HSV);			//HSVに変換(グローバル)
	hsv_color_img = smooth_img.clone();						//平滑化データをコピー

	int count = 240*320;

	//////////座標検索///////////////////////////////////////
	double min_x = -1;
	double min_y = -1;
	double max_x = -1;
	double max_y = -1;

	for(int y=0; y<240; y++)
	{
		for(int x=0; x<320; x++)
		{
			int a = hsv_img.step*y+(x*3);					//参照番号を設定
			//閾値によって抽出色以外を黒に
			if(( (hsv_img.data[a] <=5 || 175 <= hsv_img.data[a] ) && hsv_img.data[a+1] >=170 && hsv_img.data[a+2] >= 60 )){//s100/v100
				min_x = x;
				min_y = y;
				break;
			}
		}
		if ( min_x != -1 && min_y != -1 )
		{
			break;
		}
	}
	for(int y=240-1; y>=0; y--)
	{
		for(int x=320-1; x>=0; x--)
		{
			int a = hsv_img.step*y+(x*3);					//参照番号を設定
			//閾値によって抽出色以外を黒に
			if(( (hsv_img.data[a] <=5 || 175 <= hsv_img.data[a] ) && hsv_img.data[a+1] >=170 && hsv_img.data[a+2] >= 60 )){//s100/v100
				max_x = x;
				max_y = y;
				break;
			}
		}
		if ( max_x != -1 && max_y != -1 )
		{
			break;
		}
	}
	// double distance = sqrt( (max_x-min_x)*(max_x-min_x)+(max_y-min_y)*(max_y-min_y) );

	double distance = max_y-min_y;

	Debug::print(LOG_SUMMARY,"ditance = %f\r\n", distance);
	/////////////////////////////////////////////////////////



	for(int y=0; y<240;y++)
	{
		for(int x=0; x<320; x++)
		{
			int a = hsv_img.step*y+(x*3);					//参照番号を設定
			//閾値によって抽出色以外を黒に
			if(!( (hsv_img.data[a] <=5 || 175 <= hsv_img.data[a] ) && hsv_img.data[a+1] >=170 && hsv_img.data[a+2] >= 60 )){//s100/v100
				hsv_color_img.data[a] = 0;		//H値 (0-180)
				hsv_color_img.data[a+1] = 0;	//S値 (0-255)
				hsv_color_img.data[a+2] = 0;	//V値 (0-255)
				count--;
			}
		}
	}

	cvtColor(hsv_color_img, tmp_img, CV_HSV2BGR);					//一時変換
	cvtColor(tmp_img, gray_img, CV_RGB2GRAY);						//グレースケール化
	cv::threshold(gray_img, mono_img, 1, 255, CV_THRESH_BINARY);	//二値化

	moments = cv::moments(mono_img);				//重心計算
	int gX = moments.m10 / moments.m00;				//重心X位置計算
	// int gY = moments.m01 / moments.m00;				//重心Y位置計算
	//std::cout << "" << gX << "," << gY << "\n";	//重心位置をコンソールに表示
	x_gap = -160 + gX;								//中心からのX位置のずれを設定
	//画面内に重心がある時　ずれをコンソール表示
	Debug::print(LOG_SUMMARY, "color = %f%%", ((double)count / (240*320)));
	if(count > 240*320*0.0005)//閾値0.05%に変更．2014/06/14 みなと
	{
		if(-160 < x_gap && x_gap < 160)
		{
			Debug::print(LOG_SUMMARY, "gap = %d\n", x_gap);

			// もし30%以上が赤でうめつくされていたら．
			// かつ y range = 200以上
			// 今のところこの値は適当

			if ( count > 240*320*0.3 && distance > 200.0 ) 
			{
				x_gap = INT_MIN;
			}
		}
		else
		{
			Debug::print(LOG_SUMMARY, "Color is not detected.\n", x_gap);
			x_gap = INT_MAX;
		}
	}
	else //2014/06/13修正． みなと
	{
		Debug::print(LOG_SUMMARY, "Color is not detected.\n", x_gap);
		x_gap = INT_MAX;
	}

	return x_gap;										//中心からのX位置のずれを返す
}
/* ここまで　2014年6月オープンラボ前に実装 */

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
	const static double SKY_DETECT_THRESHOLD = 0.8;
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
            
			if((minH <= H && H <= maxH &&
               minS <= S && S <= maxS &&
               minV <= V && V <= maxV)
               || (H == 0 && V > 200)
			   || (V > 145 && S < 20)
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

	//Debug::print(LOG_SUMMARY, "Start\n");

	const static int DIV_NUM = 15;
	const static int PIC_SIZE_W = 320;
	const static int PIC_SIZE_H = 240;
	const static int DELETE_H_THRESHOLD = 50;
	const static double RATE = 1.6;
	const static int DIV_HOR_NUM = 5;
	const static double RISK_AVE_RATE = 0.7;

	IplImage *src_img, *dst_img1, *tmp_img;
	double risk[DIV_NUM], risk_rate[DIV_NUM];
	CvSize pic_size = cvSize(PIC_SIZE_W, PIC_SIZE_H);

	src_img = cvCreateImage(pic_size, IPL_DEPTH_8U, 1);
	cvCvtColor(pImage, src_img, CV_BGR2GRAY);
		
	tmp_img = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_16S, 1);
	dst_img1 = cvCreateImage (cvGetSize (src_img), IPL_DEPTH_8U, 1);
		
	// SobelフィルタX方向
	cvSobel (src_img, tmp_img, 1, 0, 3);
	cvConvertScaleAbs (tmp_img, dst_img1);

	// 2値化
	cvThreshold (dst_img1, dst_img1, DELETE_H_THRESHOLD, 255, CV_THRESH_BINARY);
	
	//空カット
	CvPoint pt[(DIV_HOR_NUM+1)*2+1];
	cutSky(pImage, dst_img1, pt);

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
	if(risk_ave == 0)risk_ave = 1;

	if(risk_sum == 0)risk_sum = 1;
	// 割合
	for(int i=DIV_NUM - 1; i>=0; --i){
		risk_rate[i] = risk[i] / risk_sum;
	}

	//Draw graph
	for(int i= DIV_NUM-1; i>0; --i){
		if(i>0){
			if(risk_rate[i] == 0)risk_rate[i] = 1;
			if(risk_rate[i-1] / risk_rate[i] > RATE && risk[i] > risk_ave * RISK_AVE_RATE){
				wadachi_find = true;
			}
		}
	}

	//Debug::print(LOG_SUMMARY, "ave : %f\n",risk_ave * RISK_AVE_RATE);
	//Debug::print(LOG_SUMMARY, "risk : \n");
	//for(int i=0; i<DIV_NUM; i++){
	//	Debug::print(LOG_SUMMARY, "%f\n",risk[i]);
	//}

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

	//Debug::print(LOG_SUMMARY, "Finish\n");

	return wadachi_find;
}
int ImageProc::wadachiExiting(IplImage* pImage)
{
	const static int DIV_HOR_NUM = 5;
	const static int MEDIAN = 5;
	const static int DELETE_H_THRESHOLD = 50;
	// const static int THRESHOLD_COUNT = 3;// ノイズ少のブロック数の下限
	// const static double THRESHOLD_MIN = 700000;// ノイズ数の下限

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

	// 方向決定
	/*if(count >= THRESHOLD_COUNT){
		Debug::print(LOG_SUMMARY, "Go straight\r\n");
		return 0;
	}*/
	
/*	if(0 < minNum && minNum < DIV_HOR_NUM-1){
			Debug::print(LOG_SUMMARY, "Go straight\r\n");
			return 0;
	}else{
		int ave_left = 0, ave_right = 0;
		for(int i=0; i<DIV_HOR_NUM; ++i){
			if(i <= DIV_HOR_NUM/2){
				ave_left += new_risk[i];
			}
			if(i >= DIV_HOR_NUM/2){
				ave_right += new_risk[i];
			}
		}
		ave_left /= 3; ave_right /= 3;
		if(ave_left < ave_right){
			Debug::print(LOG_SUMMARY, "Turn left\r\n");
			return -1;
		}else{
			Debug::print(LOG_SUMMARY, "Turn right\r\n");
			return 1;
		}
	}*/
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
		/* ここから　2014年6月オープンラボ前に実装 */
		else if(args[1].compare("color") == 0)
		{
			howColorGap(gCameraCapture.getFrame());
			return true;
		}
		/* ここまで　2014年6月オープンラボ前に実装 */
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
