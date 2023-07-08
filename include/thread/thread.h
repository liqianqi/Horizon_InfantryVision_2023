#pragma once
#include "../common.h"
#include "../../DaHeng/DaHengCamera.h"
#include "../../include/autoaim/inference.h"
#include "../../include/buff/base_detector_energy.h"
#include "../../include/buff/predictor.h"
// #include "../../include/autoaim/infer.h"
#include "../../include/autoaim/base_detector.h"
#include <future>
#include "../../MidVision/include/MidCamera.h"
#include "../../include/autoaim/predictor_pose.h"
#include "../../serial/Send_Receive.h"

using namespace boost::asio;
using namespace std;
using namespace Horizon;

enum BufferSize
{
    IMGAE_BUFFER = 10
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
    cv::Mat image_buffer_[50];
    double timer_buffer_[50];
    volatile unsigned int image_buffer_front_ = 0; // the produce index
    volatile unsigned int image_buffer_rear_ = 0;  // the comsum index

    void producer();

    void consumer();

    void getdata();

    Horizon::DataControler::Stm32Data TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time);

public:

    DetectorProcess infer;
    std::shared_ptr<PredictorPose> predic_pose_ = std::make_shared<PredictorPose>(); // 解算器
    std::shared_ptr<PnpSolver> pnp_solver_ = std::make_shared<PnpSolver>(yaml);

    Eigen::Vector3d coord;
    Eigen::Vector3d rotation;

    mutex serial_mutex_;

    Horizon::DataControler data_controler_;
    GimbalPose imu_data;
    int fd;

    Horizon::DataControler::VisionData visiondata; // 视觉向电控传数据
    Horizon::DataControler::Stm32Data stm32data;   // 电控向视觉发数据
    Horizon::DataControler::Stm32Data last_stm32_;

    Horizon::DataControler::Stm32Data stm32data_temp;

    CircularQueue<Horizon::DataControler::Stm32Data, 1000> stm32_deque_;
    std::deque<Horizon::DataControler::Stm32Data> MCU_data_;
    int mcu_size_ = 200;

    BUFF::DetectorProcess buff_infer;
    BUFF::BuffPredictor buffpredictor;

    GimbalPose gim;

    bool is_aim_;
};
