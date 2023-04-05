#pragma once

#include "../common.h"
#include "base_detector.h"
#include <ceres/jet.h>
#include <ceres/ceres.h>
//小装甲实际大小
static const float kRealSmallArmorWidth = 13.5;
static const float kRealSmallArmorHeight = 5.7;

//大装甲实际大小
static const float kRealLargeArmorWidth = 22.5;
static const float kRealLargeArmorHeight = 5.7;

class PnpSolver
{
public:
	PnpSolver() = delete;					// 删除默认构造函数
	PnpSolver(const string yaml);

	std::pair<Eigen::Vector3d,Eigen::Vector3d> poseCalculation(ArmorObject &obj);
private:
	cv::Mat K_;								// 内参
	cv::Mat distCoeffs_;					// 畸变系数
public:
	cv::Mat rotate_world_cam_;				// 从世界系到相机系的旋转矩阵
};

enum class ARMOR_STATE_
{
	LOSS = 0,
	INIT = 1,
	TRACK = 2,
	GYRO = 3
};
/**
 * @brief  自适应扩展卡尔曼滤波, 花山甲老师写的自适应扩展卡尔曼滤波实在优雅, 我没有信心写出更好的
 *
 * @author 上交:唐欣阳(花山甲老师)
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
        : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity()) {}

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
        P = F * P * F.transpose() + Q;
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
	Eigen::Vector3d predictTrack(Eigen::Vector3d &ptz_obj);
	Eigen::Vector3d predictGyro(Eigen::Vector3d &ptz_obj);

	void init()
	{
		init_ = true;
		loss_cnt_ = 0;
		last_state_ = ARMOR_STATE_::LOSS;
		velocities_.init();
	};
private:
	ARMOR_STATE_ state_;  // 装甲板识别状态
	ARMOR_STATE_ last_state_; // 装甲板上一帧状态
	GimbalPose imu_data_; // 云台姿态

/// @brief 状态记录变量
private:
	int loss_cnt_; // 装甲板丢失计数器，如果超过10次，记为丢失，需要初始化
	bool init_;	// 初始化开关: 触发初始化条件，装甲板切换，第一次有装甲板进入预测器, 需要初始化为true

public:
	ArmorObject ArmorSelect(std::vector<ArmorObject> &object);
	
public:
	std::shared_ptr<PnpSolver> pnp_solve_ = std::make_shared<PnpSolver>(yaml); // 解算器
	std::pair<Eigen::Vector3d, Eigen::Vector3d>	last_pose_;
	CircularQueue<Eigen::Vector3d,60> velocities_; // 速度的循环队列，方便做拟合，装甲板切换初始化
	Eigen::Vector3d last_velocity_; // 上一时刻的速度
	Eigen::Vector3d last_location_; // 上一时刻目标在云台系下的坐标
	Eigen::Vector3d CeresVelocity(CircularQueue<Eigen::Vector3d,60> velocities_); // 最小二乘法拟合速度

private:
	float v0 = 28;	// 弹速
	float bullteFlyTime(Eigen::Vector3d coord);
	GimbalPose gm_ptz;	// 角度制
	double last_time_;
	double current_time_;
};
