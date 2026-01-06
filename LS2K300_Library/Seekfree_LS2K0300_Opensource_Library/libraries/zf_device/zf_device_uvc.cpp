/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#include "zf_device_uvc.hpp"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数，初始化成员变量
// 参数说明 无
// 返回参数 无
// 使用示例 zf_device_uvc uvc_obj;
// 备注信息 初始化指针为空，摄像头状态为未打开
//-------------------------------------------------------------------------------------------------------------------
zf_device_uvc::zf_device_uvc()
    : gray_image(nullptr), rgb_image(nullptr), is_opened(false)
{

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数，释放摄像头资源
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用，无需手动调用
// 备注信息 摄像头打开时才执行释放操作，防止重复释放
//-------------------------------------------------------------------------------------------------------------------
zf_device_uvc::~zf_device_uvc()
{
    if(cap.isOpened())
    {
        cap.release();
        is_opened = false;
        gray_image = nullptr;
        rgb_image = nullptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 阻塞式等待刷新图像帧
// 参数说明 无
// 返回参数 int8 0-获取图像成功  -1-获取图像失败/帧为空/未初始化
// 使用示例 int8 res = uvc_obj.wait_image_refresh();
// 备注信息 仅采集MJPG格式图像帧，不做格式转换，提高采集效率
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::wait_image_refresh()
{
    if(!is_opened)
    {
        std::cerr << "camera not init, can not get frame!" << std::endl;
        return -1;
    }

    try 
    {
        cap >> frame_mjpg;
        if (frame_mjpg.empty()) 
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            gray_image = nullptr;
            rgb_image = nullptr;
            return -1;
        }
    } 
    catch (const cv::Exception& e) 
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        gray_image = nullptr;
        rgb_image = nullptr;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取灰度图像数据指针
// 参数说明 无
// 返回参数 uint8_t* 灰度图像首地址指针，NULL-未获取到有效图像
// 使用示例 uint8_t *p_img = uvc_obj.get_gray_image_ptr();
// 备注信息 内部完成BGR转灰度，指针指向灰度图首地址，数据格式为uint8灰度值
//-------------------------------------------------------------------------------------------------------------------
uint8_t* zf_device_uvc::get_gray_image_ptr()
{
    cv::cvtColor(frame_mjpg, frame_gray, cv::COLOR_BGR2GRAY);
    gray_image = reinterpret_cast<uint8_t*>(frame_gray.ptr(0));

    return gray_image;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取彩色图像数据指针
// 参数说明 无
// 返回参数 uint8_t* RGB彩色图像首地址指针，NULL-未获取到有效图像
// 使用示例 uint8_t *p_img = uvc_obj.get_rgb_image_ptr();
// 备注信息 内部完成BGR转BGR565，指针指向彩色图首地址，数据格式为uint16三通道连续存储
//-------------------------------------------------------------------------------------------------------------------
uint16_t* zf_device_uvc::get_rgb_image_ptr()
{
    cv::cvtColor(frame_mjpg, frame_rgb, cv::COLOR_BGR2BGR565);
    rgb_image = reinterpret_cast<uint16_t*>(frame_rgb.ptr(0));

    return rgb_image;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取摄像头当前的打开状态
// 参数说明 无
// 返回参数 bool true-已打开  false-未打开
// 使用示例 bool status = uvc_obj.isCameraOpened();
// 备注信息 直接返回摄像头状态标志位，非阻塞快速查询
//-------------------------------------------------------------------------------------------------------------------
bool zf_device_uvc::is_camera_opened() const
{
    return is_opened;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 UVC摄像头初始化配置
// 参数说明 path 摄像头设备节点路径，例如："/dev/video0"
// 返回参数 int8 0-初始化成功  -1-初始化失败
// 使用示例 int8 res = uvc_obj.uvc_camera_init("/dev/video0");
// 备注信息 配置MJPG格式、指定分辨率、帧率，打开摄像头设备
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::init(const char *path)
{
    cap.open(path);

    if(!cap.isOpened())
    {
        std::cerr << "error: no find uvc camera ." << std::endl;
        is_opened = false;
        return -1;
    } 
    else 
    {
        std::cout << "find uvc camera Successfully." << std::endl;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, UVC_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, UVC_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, UVC_FPS);

    std::cout << "get uvc width = "  << cap.get(cv::CAP_PROP_FRAME_WIDTH)  << std::endl;
    std::cout << "get uvc height = " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "get uvc fps = "    << cap.get(cv::CAP_PROP_FPS)         << std::endl;

    is_opened = true;
    return 0;
}
