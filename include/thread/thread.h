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
#include "../../serial/Send_Receive.h"
using namespace std;
using namespace Horizon;

enum BufferSize
{
    IMGAE_BUFFER = 5
};

class Factory
{
public:
    Factory()
    {
        coord[0] = 0;
        coord[1] = 0;
        coord[2] = 0;

        rotation[0] = 0;
        rotation[1] = 0;
        rotation[2] = 0;
    }
public:
    cv::Mat image_buffer_[IMGAE_BUFFER];
    double timer_buffer_[IMGAE_BUFFER];
    volatile unsigned int image_buffer_front_ = 0;   // the produce index
    volatile unsigned int image_buffer_rear_ = 0;    // the comsum index

    void producer();

    void consumer();

    void getdata();

    Horizon::DataControler::Stm32Data TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s,double src_time);

public:
    BUFF buff;
    BuffDector buffdector;
    DetectorProcess infer;
    // std::shared_ptr<PredictorPose> predic_pose_ = std::make_shared<PredictorPose>(); // 解算器
	std::shared_ptr<PnpSolver> pnp_solver_ = std::make_shared<PnpSolver>(yaml);

    Eigen::Vector3d coord;
    Eigen::Vector3d rotation;

    mutex serial_mutex_;

    Horizon::DataControler data_controler_;
    GimbalPose imu_data;
    int fd;

    Horizon::DataControler::VisionData visiondata;  // 视觉向电控传数据
    Horizon::DataControler::Stm32Data stm32data;    // 电控向视觉发数据

    CircularQueue<Horizon::DataControler::Stm32Data,200> stm32_deque_;
    std::deque<Horizon::DataControler::Stm32Data> MCU_data_;
    int mcu_size_ = 200;

};



