//大体流程差不多知道了，等摄像头到了之后，用E07，逐步尝试一下，再结合实际调试
//目前计划：1.测试图像获取2.学习/测试二值化及轮廓get（用最简单的图像）3。实际应用中的特别处理
//
//
//
#include "zf_device_uvc.h"
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

// 全局固定标定参数
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
void correct(float* camera_x,float* camera_y)
{
    // 原始图像坐标
    //camera_x = 55.5f;   
   // camera_y = 42.2f;   

    // 装原始点
    vector<Point2f> raw_pts;
    raw_pts.emplace_back(*camera_x, *camera_y);

    // 存去畸变后的点
    vector<Point2f> correct_pts;

    //坐标去畸变
    undistortPoints(raw_pts, correct_pts,
                    cameraMatrix, distCoeffs);

    printf("原始坐标 camera_x=%.2f , camera_y=%.2f\n", *camera_x, *camera_y);

    // 矫正后的坐标
    * camera_x = correct_pts[0].x;
    * camera_y = correct_pts[0].y;

    // 打印对比
    printf("矫正坐标 camera_x=%.2f , camera_y=%.2f\n", *camera_x, *camera_y);
 //   printf("矫正坐标 correct_x=%.2f , correct_y=%.2f\n", correct_x, correct_y);

}