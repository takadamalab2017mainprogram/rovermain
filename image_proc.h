#pragma once
#include <time.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "task.h"

class ImageProc : public TaskBase
{
protected:
	virtual bool onCommand(const std::vector<std::string> args);
public:
	bool isParaExist(IplImage* pImage);//画像内にパラシュートが存在するか確認する
	bool isWadachiExist(IplImage* pImage);//轍事前検知
	bool isSky(IplImage* pImage);//空の割合が一定以上なら真
	int wadachiExiting(IplImage* pImage);//-1:左 0:直進 1:右

	ImageProc();
	~ImageProc();
};

extern ImageProc gImageProc;
