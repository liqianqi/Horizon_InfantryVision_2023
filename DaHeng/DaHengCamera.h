#ifndef DAHENGCAMERA_H
#define DAHENGCAMERA_H
#include <iostream>
#include <string>
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "opencv2/opencv.hpp"


class DaHengCamera{
public:
    // 构造析构函数
    DaHengCamera();
    ~DaHengCamera();
    int StartDevice(int DeviceIndex);
    bool StartDevice();
    bool SetResolution();
    bool StreamOn();
    bool SetExposureTime();
    bool SetExposureTime(unsigned short exp_time);
    bool SetGain();
    bool SetGain(int value,int ExpGain);
    bool Set_BALANCE_AUTO(int value);
    bool Set_BALANCE();
    void getImageScale(int & width,int & height);
    bool GetMat(cv::Mat & Src);
    bool setGamma(double gamma);

private:
    //选择曝光增益通道
    enum class Channel{
        BLUE,
        GREEN,
        RED,
        ALL
    };

private:
//------------------------------------------------------------参数------------------------------------------------------------------------------
    //设备序列号1  KE0200010097
    //设备序列号2  KE0200010096
    char* pszContent = "KE0200010096\0";
    uint32_t DeviceNum = 0; // 设备号

    //控制曝光值动态变化
    unsigned short exp_time = 7000; //曝光时间
    unsigned short exp_time_value = 3000; //曝光增益

    Channel Exp_Channel = Channel::ALL;  //曝光增益通道
    unsigned short ExpGain = 10; // 曝光增益

    Channel Balance_Channel = Channel::ALL;  //白平衡
    float Balence_value = 40; //白平衡系数


    uint32_t ScaleReduction = 3; //画面缩小系数
//------------------------------------------------------------参数------------------------------------------------------------------------------

private:

//------------------------------------------------------------参数------------------------------------------------------------------------------

public:
    GX_STATUS status = GX_STATUS_SUCCESS; //状态指标
    //GX_STATUS emStatus = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL; //设备地址
    GX_OPEN_PARAM streamOpenParam;  //图像流
    PGX_FRAME_BUFFER pFrameBuffer;

};

#endif
