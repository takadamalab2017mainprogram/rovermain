#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <ctime>
#include <algorithm>

#define PI 3.14159265

using namespace std;

double pix_to_xy(int pixel, float pos[2], int width) {
	int pix_x = pixel % width;
	int pix_y = pixel / width;
	double dis = pow(pow(pix_x - pos[0], 2) + pow(pix_y - pos[1], 2), 0.5);
	return dis;
}

double * xy_to_ang(float xy[2], int width, int height) {
	//左上角为原点的直角坐标系转中心为原点的极坐标系
	float xx = xy[0] - width / 2;
	float yy = height / 2 - xy[1];
	float rad = pow(pow(xx, 2) + pow(yy, 2), 0.5);
	double ang = atan(yy / xx);
	if (ang > 0 & xx < 0) {
		ang += PI;
	}
	else if (ang < 0 & yy > 0) {
		ang += PI;
	}
	double polar[2] = { rad, ang };
	return polar;
}

double * ang_to_xy(double radang[2], int width, int height) {
	//以中心为原点的极坐标系转换为以左上角为原点的直角坐标系
	double yy = sin(radang[1]) * radang[0];
	double xx = cos(radang[1]) * radang[0];
	double y = height / 2 - yy;
	double x = xx + width / 2;
	double pos[2] = { x, y };
	return pos;

}

float point_to_line(int point[2], float p1[2], float p2[2]) {
	//xy座標の点とｐ1、ｐ２二点からなしたlineの距離
	float dis = 0.f;

	float dx = p2[0] - p1[0];
	float dy = p2[1] - p1[1];

	// 两直线垂直，向量表示法，转换后公示
	float k = -((p1[0] - point[0])*dx + (p1[1] - point[1])*dy) / (dx*dx + dy*dy);
	float footX = k*dx + p1[0];
	float footY = k*dy + p1[1];

	//if垂足是否落在线段上
	if (footY >= min(p1[1], p2[1]) && footY <= max(p1[1], p2[1])
		&& footX >= min(p1[0], p2[0]) && footX <= max(p1[0], p2[0]))
	{
		dis = sqrtf((footX - point[0])*(footX - point[0]) + (footY - point[1])*(footY - point[1]));
	}
	else
	{
		float dis1 = sqrtf((p1[0] - point[0])*(p1[0] - point[0]) + (p1[1] - point[1])*(p1[1] - point[1]));
		float dis2 = sqrtf((p2[0] - point[0])*(p2[0] - point[0]) + (p2[1] - point[1])*(p2[1] - point[1]));

		dis = (dis1 < dis2 ? dis1 : dis2);
	}

	return dis;
}