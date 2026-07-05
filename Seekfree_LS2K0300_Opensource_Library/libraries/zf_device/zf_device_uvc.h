#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h


#include "zf_common_typedef.h"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>  // for cv::cvtColor
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#include <opencv2/opencv.hpp>

#include <vector>
#include <cstdio>
#include <iostream> // for std::cerr
#include <fstream>  // for std::ofstream
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <opencv2/opencv.hpp>

#define UVC_WIDTH   160
#define UVC_HEIGHT  120
#define UVC_FPS     15

int uvc_camera_init(const char *path);
int wait_image_refresh();



extern uint8_t *rgay_image;

extern cv::Mat frame_rgb;      
extern cv::Mat frame_rgay;     

extern int x,y;//传出坐标
#endif