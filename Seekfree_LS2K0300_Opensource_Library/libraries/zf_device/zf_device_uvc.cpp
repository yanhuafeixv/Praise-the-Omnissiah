#include "zf_device_uvc.h"
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
#include "zf_device_uvc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
// #include "camera.h"


using namespace cv;
using namespace std;



Mat cameraMatrix = (Mat_<double>(3,3) <<
    75.2345, 0.0,    80.0,
    0.0,    75.2345, 60.0,
    0.0,     0.0,     1.0
);

Mat distCoeffs = (Mat_<double>(5,1) <<
    0.0456,
   -0.0123,
    0.0011,
    0.0007,
    0.0000
);


cv::Mat frame_rgb;      // 构建opencv对象 彩色
cv::Mat frame_rgay;     // 构建opencv对象 灰度
cv::Mat frame_blur;     // 构建opencv对象 高斯滤波
cv::Mat frame_hold;     // 构建opencv对象 二值化
cv::Mat frame_hsv;     // 构建opencv对象 hsv
cv::Mat frame_m;     // 构建opencv对象 

uint8_t *rgay_image;    // 灰度图像数组指针

VideoCapture cap;

int uvc_camera_init(const char *path)
{
    cap.open(path);

    if(!cap.isOpened())
    {
        printf("find uvc camera error.\r\n");
        return -1;
    } 
    else 
    {
        printf("find uvc camera Successfully.\r\n");
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));  // 设置格式
    cap.set(CAP_PROP_FRAME_WIDTH, UVC_WIDTH);                           // 设置摄像头宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, UVC_HEIGHT);                         // 设置摄像头高度
    cap.set(CAP_PROP_FPS, UVC_FPS);                                     // 显示屏幕帧率

    printf("get uvc width = %f.\r\n",  cap.get(CAP_PROP_FRAME_WIDTH));
    printf("get uvc height = %f.\r\n", cap.get(CAP_PROP_FRAME_HEIGHT));
    printf("get uvc fps = %f.\r\n",    cap.get(CAP_PROP_FPS));

    return 0;
}


int wait_image_refresh()
{
    try 
    {
        // 阻塞式等待图像刷新
        cap >> frame_rgb;
        // cap.read(frame_rgb);
        if (frame_rgb.empty()) 
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            return -1;
        }
    } 
    catch (const cv::Exception& e) 
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    /*写-调*/
    // rgb转灰度
 //   cv::cvtColor(frame_rgb, frame_rgay, cv::COLOR_BGR2GRAY);


    cv::cvtColor(frame_rgb, frame_hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower = cv::Scalar(0, 120, 70);
    cv::Scalar upper = cv::Scalar(10, 255, 255);


//高斯滤波
 //   cv::GaussianBlur(frame_rgay,frame_blur,cv::Size(3,3),0);//！！@
  //  cv::equalizeHist(frame_blur, frame_blur);
//    cv::adaptiveThreshold(frame_blur, frame_hold,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV,31,6);
   // cv::threshold(frame_blur, frame_hold, 110, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //二值化
 //   cv::threshold(frame_blur,frame_hold,110,255,cv::THRESH_BINARY);//！！@


    cv::inRange(frame_hsv, lower, upper, frame_m);//@!!
    //开运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
 //   cv::morphologyEx(frame_hold, result, cv::MORPH_OPEN, kernel);//！！@
    cv::Mat result;
    cv::morphologyEx(frame_m, result, cv::MORPH_OPEN, kernel);//！！@
//    cv::morphologyEx(result,result, cv::MORPH_CLOSE, kernel);

 //   cv::cvtColor(result, frame_rgay, cv::COLOR_BGR2GRAY);
    //找轮廓
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 遍历所有轮廓
     for(size_t i=0;i<contours.size();i++)
    {
        //通过面积去掉小轮廓
        double area = cv::contourArea(contours[i]);
        if(area < 150) continue;  //！！@

       //显示：原图上，中心十字+坐标+框框？
     //   cv::Rect rect = cv::boundingRect(contours[i]);
        cv::RotatedRect min_rect = cv::minAreaRect(contours[i]);
        //中心坐标
      /*  int cx = rect.x + rect.width/2;
        int cy = rect.y + rect.height/2;

       cv::rectangle(frame_rgay,rect,cv::Scalar(0,255,0),2);
        cv::circle(frame_rgay,cv::Point(cx,cy),4,cv::Scalar(0,0,255),-1);
        char text[32];
        sprintf(text, "(%d, %d)", cx, cy);
        cv::putText(frame_rgay, text,cv::Point(cx - 30, cy - 10), cv::FONT_HERSHEY_SIMPLEX,0.4,
        cv::Scalar(0,255,255), 1 );
        */
            float cx = min_rect.center.x;
            x=cx;
            float cy = min_rect.center.y;
            y=cy;

            //去畸变
            vector<Point2f> raw_pts;
            raw_pts.emplace_back(cx,cy);

            // 存去畸变后的点
            vector<Point2f> correct_pts;

            // 核心：单点坐标去畸变
            undistortPoints(raw_pts, correct_pts,
                           cameraMatrix, distCoeffs);

            // 取出矫正后的坐标
            cx = correct_pts[0].x;
            cy = correct_pts[0].y;

        //    correct(&cx,&cy);//摄像头标定去畸变
            x=(int)cx;
            y=(int)cy;//赋值给全局变量

            //下面是显示
            cv::Point2f corners[4];
            min_rect.points(corners);
            for (int j = 0; j < 4; j++)
            {
                cv::line(frame_rgb, corners[j], corners[(j+1)%4], cv::Scalar(0,255,0), 2);
            }

            // 画中心点 + 显示坐标（完全不变！）
            cv::circle(frame_rgb, cv::Point(cx, cy), 4, cv::Scalar(0,0,255), -1);
            char text[32];
            sprintf(text, "(%f, %f)", cx, cy);
            cv::putText(frame_rgb, text, cv::Point(cx - 30, cy - 10), 
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
            
        
    }
        
   // cv对象转指针
   //  rgay_image = reinterpret_cast<uint8_t *>(frame_hold.ptr(0));
   /*7.5注释掉
    cv::cvtColor(frame_rgb, frame_rgay, cv::COLOR_BGR2GRAY);
    rgay_image = reinterpret_cast<uint8_t *>( frame_rgay.ptr(0));
    */

    return 0;
}