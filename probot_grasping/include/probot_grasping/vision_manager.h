#ifndef PROBOT_VISION_MANAGER
#define PROBOT_VISION_MANAGER

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

typedef enum {
	my_BLUE = 0,
	my_GREEN = 1,
	my_RED = 2
} color_t;

class VisionManager
{
  public:
	/**
	 * @brief 构造函数
	 * 
	 * @param length 桌子长度
	 * @param width 桌子宽度
	 */
	VisionManager(float length, float width);
	/**
	 * @brief 获取物块坐标
	 * 
	 * @param img 图像
	 * @param x 读取到的x坐标
	 * @param y 读取到的y坐标
	 * @param color 颜色判断
	 */
	void GetLocation(cv::Mat img, float &x, float &y, int color);

  private:

	/**
	 * @brief 物体检测
	 * 
	 * @param pixel_x 检测到的x坐标
	 * @param pixel_y 检测到的y坐标
	 * @param tablePos 桌面位置
	 * @param color 颜色判断
	 */
	void DetectObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos, int color);

	void TransToCamera(float &pixel_mm_x, float &pixel_mm_y);

	void DetectTable(cv::Rect &tablePos);

	float pixels_permm_x;
	float pixels_permm_y;

	float curr_pixel_centre_x;
	float curr_pixel_centre_y;

	float table_length;
	float table_width;

	float img_centre_x_;
	float img_centre_y_;

	cv::Mat curr_img;
};

#endif
