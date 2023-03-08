#pragma once
#include "../common.h"
#include "../../DaHeng/DaHengCamera.h"
#include "../../include/autoaim/inference.h"
#include "../../include/buff/buff.h"
//#include "../../include/autoaim/infer.h"
#include "../../include/autoaim/base_detector.h"
#include <future>
#include "../../MidVision/include/MidCamera.h"
#include "../../include/autoaim/predictor_pose.h"
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
	std::shared_ptr<PnpSolver> pnp_solver_ = std::make_shared<PnpSolver>(yaml); // 解算器

};



