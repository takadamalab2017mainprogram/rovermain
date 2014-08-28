#pragma once
#include <time.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "task.h"

class ImageProc : public TaskBase
{
	unsigned int mHMinThreshold;
	unsigned int mHMaxThreshold;
	unsigned int mSMinThreshold;
	unsigned int mVMinThreshold;
	double mDistanceThreshold;
	
protected:
	virtual bool onCommand(const std::vector<std::string> args);
public:
	int howColorGap(IplImage* pImage);//画像中心から特定の色重心がどれだけずれているか
	bool isParaExist(IplImage* pImage);//画像内にパラシュートが存在するか確認する
	bool isWadachiExist(IplImage* pImage);//轍事前検知
	bool isSky(IplImage* pImage);//空の割合が一定以上なら真
	void cutSky(IplImage* pSrc,IplImage* pDest, CvPoint* pt);//pDestの空部分を塗りつぶす
	int wadachiExiting(IplImage* pImage);//-1:左 0:直進 1:右

	ImageProc();
	~ImageProc();
};

extern ImageProc gImageProc;
