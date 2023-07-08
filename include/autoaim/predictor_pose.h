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
	TRACK = 1
};

struct Predict {
    /*
     * 此处定义匀速直线运动模型
     */
    template<class T>
    void operator()(const T* x0, T* x1) {    // x0[6],x1[6]
        x1[0] = x0[0] + delta_t * x0[1];  //0.1, x
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1, y
        x1[3] = x0[3];  //100
        x1[4] = x0[4] + delta_t * x0[5];  //0.1, z 
        x1[5] = x0[5];  //100
    }

    double delta_t;
};

template<class T>
void xyz2pyd(T* xyz, T* pyd)  // xyz[3], pyd[3]
{
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[1], ceres::sqrt(xyz[0]*xyz[0]+xyz[2]*xyz[2]));  // pitch
    pyd[1] = ceres::atan2(xyz[0], xyz[2]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
}

struct Measure 
{
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T* x, T* y) { // x[6], y[3]
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};

/**
 * @brief  自适应扩展卡尔曼滤波, 花山甲老师写的自适应扩展卡尔曼滤波实在优雅, 我没有信心写出更好的，baipiao(bushi)
 *
 * @author 上交:唐欣阳(花山甲老师yyds)
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
        std::cout << P << std::endl;
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
        std::cout << F * P * F.transpose() << std::endl;
        P = F * P * F.transpose() + Q;
        std::cout << "predict variables is x: " << Xp[0] << " y: " << Xp[2] << " z: " << Xp[4] << std::endl; 

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

        std::cout << "update variables is x: " << Xe[0] << " y: " << Xe[2] << " z: " << Xe[4] << std::endl; 
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
	// cam_coord是相机坐标系坐标，rotate_world_cam是世界坐标系到相机坐标系旋转
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
	
public:
	std::shared_ptr<PnpSolver> pnp_solve_ = std::make_shared<PnpSolver>(yaml); // 解算器
	std::pair<Eigen::Vector3d, Eigen::Vector3d>	last_pose_;
	std::deque<Eigen::Vector4d> velocities_; // 速度的循环队列，方便做拟合，装甲板切换初始化
	Eigen::Vector3d last_velocity_; // 上一时刻的速度
	Eigen::Vector3d last_location_; // 上一时刻目标在云台系下的坐标
	Eigen::Vector3d CeresVelocity(std::deque<Eigen::Vector4d> velocities); // 最小二乘法拟合速度
	int velocities_deque_size_ = 20;

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
    AdaptiveEKF<6, 3> ekf;  // 创建ekf
    float move_ = 0;
    GimbalPose last_eular_;

public:
    std::deque<Eigen::Vector4d> yaw_rad_; // 投入4个，但是目前只要1个
    int yaw_rad_size_ = 30;
    bool is_gyro_ = false;
    Eigen::Vector3d last_posture_;
    double threshold_yaw_rad_ = 60;
    double R_ = 0.6;

//**************逻辑判断***************//
    int switch_count_ = 0;
    int last_obj_nums = 0;
    float last_id;
    ArmorObject last_obj;
    bool flag = false;
    int v_count = 0;
//**************逻辑判断***************//

//**************反陀螺1***************//
    Kalman_Filter kf;
    Vector3d last_yaw_v;
//**************反陀螺1***************//

//**************反陀螺2***************//
    int cnt_switch = 0;
    Vector3d left_switch_;
    Vector3d right_switch_;

    std::deque<double> time_buff_;

    float last_switch_time_;
    int is_gyro_cnt_ = 0;
    bool isSwitch(Vector3d up_switch, Vector3d down_switch);
    bool isGyro(std::deque<double> time_buff_, Vector3d up_switch, Vector3d down_switch);

    float time_diff = 0;
//**************反陀螺2***************//

    Eigen::Vector3d cam3ptz(GimbalPose gm,Vector3d pos);

};
