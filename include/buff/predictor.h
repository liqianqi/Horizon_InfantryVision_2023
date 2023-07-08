#pragma once
#include "base_detector_energy.h"
#include <ceres/jet.h>
#include <ceres/ceres.h>
namespace BUFF
{
static const float RealSmallArmorWidth = 17.0;
static const float RealSmallArmorHeight = 15.5;

const static string yaml = "../param/camera_info.yaml";

//大装甲实际大小
static const float RealLargeArmorWidth = 22.5;
static const float RealLargeArmorHeight = 5.7;

class PnpSolver_
{
public:
	PnpSolver_() = delete;					// 删除默认构造函数
	PnpSolver_(const string yaml);

	std::pair<Eigen::Vector3d,Eigen::Vector3d> poseCalculation(BuffObject &obj);
    Eigen::Vector3d get_euler_angle(cv::Mat rotation_vector);
public:
	cv::Mat K_;								// 内参
	cv::Mat distCoeffs_;					// 畸变系数
public:
	cv::Mat rotate_world_cam_;				// 从世界系到相机系的旋转矩阵
    bool is_large_;
};

class BuffPredictor
{
private:
    struct CURVE_FITTING_COST
    {
        CURVE_FITTING_COST (double x, double y) : _x ( x ), _y ( y ) {}
        // 残差的计算
        template <typename T>
        bool operator() (
            const T* params,     // 模型参数，有3维
            T* residual) const     // 残差
        {
            residual[0] = T (_y) - params[0] * ceres::sin(params[1] * T (_x) + params[2]) - params[3]; // f(x) = a * sin(ω * t + θ) + b
            return true;
        }
        const double _x, _y;    // x,y数据

    };
    struct CURVE_FITTING_COST_PHASE
    {
        CURVE_FITTING_COST_PHASE (double x, double y, double a, double omega, double dc) : _x (x), _y (y), _a(a), _omega(omega), _dc(dc){}
        // 残差的计算
        template <typename T>
        bool operator() (
        const T* phase,     // 模型参数，有1维
        T* residual) const     // 残差
        {
            residual[0] = T (_y) - T (_a) * ceres::sin(T(_omega) * T (_x) + phase[0]) - T(_dc); // f(x) = a * sin(ω * t + θ)
            return true;
        }
        const double _x, _y, _a, _omega, _dc;    // x,y数据
    };

    //目标信息
    struct TargetInfo
    {
        double speed;
        Eigen::Vector3d coord;
        int timestamp;
        double theata;
    };

private:
    double params[4];
    double bullet_speed = 28;                                            
    std::deque<TargetInfo> history_info;                                    //目标队列
    const int max_timespan = 20000;                                         //最大时间跨度，大于该时间重置预测器(ms)
    const double max_rmse = 0.4;                                               //TODO:回归函数最大Cost
    const int max_v = 3;                                                  //设置最大速度,单位rad/s
    const int max_a = 8;                                                  //设置最大角加速度,单位rad/s^2
    const int history_deque_len_cos = 250;                                  //大符全部参数拟合队列长度
    const int history_deque_len_phase = 100;                                  //大符相位参数拟合队列长度
    const int history_deque_len_uniform = 100;                                  //小符转速求解队列长度
    const int delay_small = 175;                                                  //小符发弹延迟
    const int delay_big = 100;                                              //大符发弹延迟
    const int window_size = 2;                                              //滑动窗口大小

public:
    TargetInfo last_target;                                                  //最后目标
    int mode;                                                               //预测器模式，0为小符，1为大符
    int last_mode;
    bool is_params_confirmed;
    std::shared_ptr<PnpSolver_> pnp = std::make_shared<PnpSolver_>(yaml); // 解算器
    Eigen::Matrix3d transform_vector_;

    std::deque<Eigen::Vector3d> R_QUENE;
    int r_size = 35;
    Eigen::Vector3d R_use;
    std::deque<Eigen::Vector2d> rolls;
    int roll_size = 20;
    GimbalPose gm_ptz;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> last_coord_ptz_; 
public:
    GimbalPose run(double src_time,std::vector<BuffObject> objects, GimbalPose imu, int flag = 0);

    Eigen::Vector3d getR(Eigen::Vector3d coord, double roll);
    Eigen::Vector3d slide_windows_R(std::deque<Eigen::Vector3d> R_QUENE)
    {
        double x,y,z;
        for(int i = 0; i < R_QUENE.size(); i++)
        {
            x += R_QUENE[i][0];
            y += R_QUENE[i][1];
            z += R_QUENE[i][2];
        }

        x = x/R_QUENE.size();
        y = y/R_QUENE.size();
        z = z/R_QUENE.size();

        return Eigen::Vector3d{x,y,z};
    }
    float bullteFlyTime(Eigen::Vector3d coord);
    double CeresRads(std::deque<Eigen::Vector2d> rolls);

    BuffPredictor(){};
    ~BuffPredictor(){};
    bool predict(double speed, double dist, int timestamp, double &result);
    double calcAimingAngleOffset(double params[4], double t0, double t1, int mode);
    double shiftWindowFilter(int start_idx);
    bool setBulletSpeed(double speed);
    double evalRMSE(double params[4]);
    double evalMAPE(double params[4]);
    
    Eigen::Vector3d cam3ptz(GimbalPose gm, Eigen::Vector3d pos)
    {
        pos[0] = pos[0] + X_BIAS;
        pos[1] = pos[1] + Y_BIAS;
        pos[2] = pos[2] + Z_BIAS;
        //pos = pos.transpose();

        Eigen::Matrix3d pitch_rotation_matrix_;
        Eigen::Matrix3d yaw_rotation_matrix_;

        
        std::cout << "camera pose " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
        std::cout << "solve gm.pitch" << gm.pitch << std::endl;
        std::cout << "solve gm.yaw" << gm.yaw << std::endl;

        pitch_rotation_matrix_
            << 1,
            0, 0,
            0, std::cos((gm.pitch) * (CV_PI / 180)), std::sin((gm.pitch) * (CV_PI / 180)),
            0, -std::sin((gm.pitch) * (CV_PI / 180)), std::cos((gm.pitch) * (CV_PI / 180));

        yaw_rotation_matrix_
            << std::cos((gm.yaw) * (CV_PI / 180)),
            0, std::sin((gm.yaw) * (CV_PI / 180)),
            0, 1, 0,
            -std::sin((gm.yaw) * (CV_PI / 180)), 0, std::cos((gm.yaw) * (CV_PI / 180));

        Eigen::Vector3d t_pos_ =  yaw_rotation_matrix_ * pitch_rotation_matrix_ * pos;
     
        std::cout << "world pose " << t_pos_[0] << " " << t_pos_[1] << " " << t_pos_[2] << std::endl;

        Eigen::Matrix3d transform_vector;
        transform_vector = yaw_rotation_matrix_ * pitch_rotation_matrix_;
        transform_vector_ = transform_vector.inverse();

        return t_pos_;
    }

};
}