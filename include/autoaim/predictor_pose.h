#pragma once

#include "../common.h"
#include "base_detector.h"
#include <ceres/jet.h>
#include <ceres/ceres.h>
#include "kf.h"

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED1 "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE1 "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

//小装甲实际大小
static const float kRealSmallArmorWidth = 13.5;
static const float kRealSmallArmorHeight = 5.7;

//大装甲实际大小
static const float kRealLargeArmorWidth = 22.5;
static const float kRealLargeArmorHeight = 5.7;
using namespace Eigen;
class PnpSolver
{
public:
	PnpSolver() = delete;					// 删除默认构造函数
	PnpSolver(const string yaml);

	std::pair<Eigen::Vector3d,Eigen::Vector3d> poseCalculation(ArmorObject &obj);
public:
	cv::Mat K_;								// 内参
	cv::Mat distCoeffs_;					// 畸变系数
public:
	cv::Mat rotate_world_cam_;				// 从世界系到相机系的旋转矩阵
    bool is_large_;

};

enum class ARMOR_STATE_
{
	LOSS = 0,
	TRACK = 1,
	TRANSLATE = 2,
	GYRO = 3,
	LOST_REID = 4
};

/**
 * @brief 四装甲板整车运动模型的状态预测
 * 
 * 状态向量 (11维):
 * x[0]  - center_x        中心 x 坐标
 * x[1]  - center_vx       中心 x 速度
 * x[2]  - center_y        中心 y 坐标 (组0/2装甲板的高度)
 * x[3]  - center_vy       中心 y 速度
 * x[4]  - center_z        中心 z 坐标
 * x[5]  - center_vz       中心 z 速度
 * x[6]  - phase           装甲板0的相位角
 * x[7]  - omega           角速度 (rad/s)
 * x[8]  - r1              组0/2装甲板的旋转半径 (短半径)
 * x[9]  - r2              组1/3装甲板的旋转半径 (长半径)
 * x[10] - delta_y         组1/3相对组0/2的高度差
 * 
 * 装甲板布局 (俯视图，逆时针):
 * - 装甲板0: phase = θ,        radius = r1, y = center_y
 * - 装甲板1: phase = θ + π/2,  radius = r2, y = center_y + delta_y
 * - 装甲板2: phase = θ + π,    radius = r1, y = center_y
 * - 装甲板3: phase = θ + 3π/2, radius = r2, y = center_y + delta_y
 */
struct SpinPredict {
    template<class T>
    void operator()(const T* x0, T* x1) {
        x1[0] = x0[0] + T(delta_t) * x0[1];   // center x
        x1[1] = x0[1];                         // center vx
        x1[2] = x0[2] + T(delta_t) * x0[3];   // center y
        x1[3] = x0[3];                         // center vy
        x1[4] = x0[4] + T(delta_t) * x0[5];   // center z
        x1[5] = x0[5];                         // center vz
        x1[6] = x0[6] + T(delta_t) * x0[7];   // phase
        x1[7] = x0[7];                         // omega
        x1[8] = x0[8];                         // r1 (组0/2半径)
        x1[9] = x0[9];                         // r2 (组1/3半径)
        x1[10] = x0[10];                       // delta_y (高度差)
    }

    double delta_t;
};

/**
 * @brief 四装甲板整车运动模型的观测方程
 * 
 * 根据当前跟踪的装甲板组别选择对应的半径和高度:
 * - armor_group = 0: 使用 r1, y = center_y (装甲板0或2)
 * - armor_group = 1: 使用 r2, y = center_y + delta_y (装甲板1或3)
 */
struct SpinMeasure {
    template<class T>
    void operator()(const T* x, T* y) {
        T phase_offset = T(armor_group) * T(CV_PI / 2.0);
        T current_phase = x[6] + phase_offset;
        T radius = (armor_group == 0 || armor_group == 2) ? x[8] : x[9];
        T height_offset = (armor_group == 0 || armor_group == 2) ? T(0) : x[10];
        
        y[0] = x[0] + radius * ceres::cos(current_phase);
        y[1] = x[2] + height_offset;
        y[2] = x[4] + radius * ceres::sin(current_phase);
    }
    
    int armor_group = 0;  // 0=装甲板0/2, 1=装甲板1/3
};

/**
 * @brief  自适应扩展卡尔曼滤波,
 *
 * @author 上交:唐欣阳
 */
template <int N_X, int N_Y>
class AdaptiveEKF
{
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero())
        : Xe(X0), P(MatrixXX::Identity())
    {
        VISION_PREDICTOR_STREAM << P << std::endl;
        Q.setIdentity();
        R.setIdentity();

        if constexpr (N_X == 6 && N_Y == 3)
        {
            Q << 0.15 ,0,    0,     0,  0,  0,
                 0,    1,  0,     0,  0,  0,
                 0,    0,    0.1,   0,  0,  0,
                 0,    0,    0,    0.3, 0,  0,
                 0,    0,    0,     0,  0.5,0,
                 0,    0,    0,     0,  0,  1;

            R << 4,0,0,
                 0,1,0,
                 0,0,0.5;
        }
        else
        {
            Q *= 0.05;
            R *= 0.02;
        }
    }
    
    void init(const VectorX &X0 = VectorX::Zero()) 
    {
        Xe = X0;
    }

    template <class Func>
    VectorX predict(Func &&func)
    {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];

        for (int i = 0; i < N_X; i++)
        {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }

        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < N_X; i++)
        {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        VISION_PREDICTOR_STREAM << F * P * F.transpose() << std::endl;
        P = F * P * F.transpose() + Q;
        VISION_PREDICTOR_STREAM << "predict variables is x: " << Xp[0] << " y: " << Xp[2] << " z: " << Xp[4] << std::endl; 

        return Xp;
    }

    template <class Func>
    VectorX update(Func &&func, const VectorY &Y)
    {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++)
        {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y; i++)
        {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        Xe = Xp + K * (Y - Yp);
        P = (MatrixXX::Identity() - K * H) * P;

        VISION_PREDICTOR_STREAM << "update variables is x: " << Xe[0] << " y: " << Xe[2] << " z: " << Xe[4] << std::endl; 
        return Xe;
    }

    VectorX Xe; // 估计状态变量
    VectorX Xp; // 预测状态变量
    MatrixXX F; // 预测雅克比
    MatrixYX H; // 观测雅克比
    MatrixXX P; // 状态协方差
    MatrixXX Q; // 预测过程协方差
    MatrixYY R; // 观测过程协方差
    MatrixXY K; // 卡尔曼增益
    VectorY Yp; // 预测观测量
};

const static string yaml = "../param/camera_info.yaml";

class PredictorPose
{
public:
	PredictorPose()
	{
		init_ = true;
		loss_cnt_ = 0;
	};

private:
	// 相机系转云台系，转到IMU上没多大作用
	// 转出来的分别是装甲板相对于云台的位置和姿态
	// cam_coord是相机坐标系坐标, rotate_world_cam是世界坐标系到相机坐标系旋转
	std::pair<Eigen::Vector3d,Eigen::Vector3d> cam2ptz(Eigen::Vector3d &cam_coord, cv::Mat &rotate_world_cam);

public:
	GimbalPose run(GimbalPose &imu_data, std::vector<ArmorObject> &objects, double time);

private:

	void init()
	{
		init_ = true;
		loss_cnt_ = 0;
		last_state_ = ARMOR_STATE_::LOSS;
	};
public:
	ARMOR_STATE_ state_ = ARMOR_STATE_::LOSS;  // 装甲板识别状态
	ARMOR_STATE_ last_state_; // 装甲板上一帧状态
	GimbalPose imu_data_; // 云台姿态

/// @brief 状态记录变量
private:
	int loss_cnt_; // 装甲板丢失计数器，如果超过10次，记为丢失，需要初始化
	bool init_;	// 初始化开关: 触发初始化条件，装甲板切换，第一次有装甲板进入预测器，需要初始化为true

public:
	ArmorObject ArmorSelect(std::vector<ArmorObject> &object);
	ArmorObject ArmorSelectByPrediction(std::vector<ArmorObject> &objects, bool reidentify);
	
public:
	std::shared_ptr<PnpSolver> pnp_solve_ = std::make_shared<PnpSolver>(yaml); // 解算器
	std::pair<Eigen::Vector3d, Eigen::Vector3d>	last_pose_;
	Eigen::Vector3d last_velocity_; // 上一时刻的速度
	Eigen::Vector3d last_location_; // 上一时刻目标在云台系下的坐标

    Eigen::Matrix3d transform_vector_;
    Eigen::Vector3d predict_location_;
public:
	float v0 = 28;	// 弹速
	float bullteFlyTime(Eigen::Vector3d coord);
	GimbalPose gm_ptz;	// 角度制
	double last_time_;
	double current_time_;
    cv::Point2f obj_pixe_;
public:
    AdaptiveEKF<11, 3> spin_ekf;  // 四装甲板整车模型: 中心位置/速度、相位、角速度、两个半径、高度差
    int current_armor_group_ = 0;  // 当前跟踪的装甲板组 (0=组0/2, 1=组1/3)
    float move_ = 0;
    GimbalPose last_eular_;
    bool is_gyro_ = false;

public:
    int last_obj_nums = 0;
    float last_id;
    ArmorObject last_obj;
    bool flag = false;

    int cnt_switch = 0;
    Vector3d left_switch_;
    Vector3d right_switch_;
    float last_switch_time_;
    int is_gyro_cnt_ = 0;
    bool isSwitch(Vector3d up_switch, Vector3d down_switch);
    float time_diff = 0;

    Eigen::Vector3d cam3ptz(GimbalPose gm,Vector3d pos);

private:
    Eigen::Vector3d spinArmorPosition(const Eigen::Matrix<double, 11, 1> &state, double dt, int armor_group) const;
    Eigen::Vector3d spinCenterPosition(const Eigen::Matrix<double, 11, 1> &state, double dt) const;
    Eigen::Vector3d runSpinEkf(const Eigen::Vector3d &measured_armor, double dt, bool allow_update);
    void resetSpinEkf(const Eigen::Vector3d &measured_armor, double time, double r1 = 0.0, double r2 = 0.0, double delta_y = 0.0);
    void updateSpinMode(bool armor_switched, double dt);
    void initSpinFromSwitch(const Eigen::Vector3d &current_armor, const Eigen::Vector3d &last_armor, double dt);
    int detectArmorGroup(const Eigen::Vector3d &armor_pos) const;  // 检测装甲板属于哪组

};
