#pragma once
#include "../common.h"
#include "../../DaHeng/DaHengCamera.h"
#include "../../include/autoaim/inference.h"
#include "../../include/buff/buff.h"
//#include "../../include/autoaim/infer.h"
#include "../../include/autoaim/base_detector.h"
#include <future>
using namespace std;

enum BufferSize
{
    IMGAE_BUFFER = 5
};

class Factory
{
public:
    Factory(){}
public:
    cv::Mat image_buffer_[IMGAE_BUFFER];
	double timer_buffer_[IMGAE_BUFFER];
    volatile unsigned int image_buffer_front_ = 0;   // the produce index
    volatile unsigned int image_buffer_rear_ = 0;    // the comsum index

    void producer();

    void consumer();
    mutex image_mutex_;               //数据上🔓
    BUFF buff;
    BuffDector buffdector;
    DetectorProcess infer;
    //vector<Detector::Object> detected_objects;


//    Detector infer1;
//    vector<Object> ArmorObjects;

};



