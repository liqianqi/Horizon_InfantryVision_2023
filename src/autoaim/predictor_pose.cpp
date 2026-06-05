#include "../../include/autoaim/predictor_pose.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kMinDt = 1e-3;
constexpr double kMaxDt = 0.2;
constexpr double kLostReidDistanceGate = 1.2;
constexpr double kGyroEnterOmega = 1.5; // rad/s
constexpr double kGyroExitOmega = 0.7;  // rad/s
constexpr double kMinSpinRadius = 0.08;
constexpr double kMaxSpinRadius = 0.8;

double clampDt(double dt)
{
	if (!std::isfinite(dt) || dt < kMinDt)
	{
		return kMinDt;
	}
	return std::min(dt, kMaxDt);
}

double normalizeAngle(double angle)
{
	while (angle > CV_PI)
	{
		angle -= 2.0 * CV_PI;
	}
	while (angle < -CV_PI)
	{
		angle += 2.0 * CV_PI;
	}
	return angle;
}

}

PnpSolver::PnpSolver(const string yaml)
{
	cv::FileStorage fs(yaml, cv::FileStorage::READ);
	fs["M1"] >> K_;
	fs["D1"] >> distCoeffs_;
	fs.release();
}

/**
 * @brief 将旋转矩阵转化为欧拉角
 * @param R 旋转矩阵
 * @return 欧拉角
 */
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	bool singular = sy < 1e-6;
	double x, y, z;
	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}
	return {z, y, x};
}

Eigen::Vector3d get_euler_angle(cv::Mat rotation_vector)
{
	double theta = cv::norm(rotation_vector, cv::NORM_L2);

	double w = std::cos(theta / 2);
	double x = std::sin(theta / 2) * rotation_vector.ptr<double>(0)[0] / theta;
	double y = std::sin(theta / 2) * rotation_vector.ptr<double>(1)[0] / theta;
	double z = std::sin(theta / 2) * rotation_vector.ptr<double>(2)[0] / theta;

	double ysqr = y * y;
	// pitch (x-axis rotation)
	double t0 = 2.0 * (w * x + y * z);
	double t1 = 1.0 - 2.0 * (x * x + ysqr);
	double pitch = std::atan2(t0, t1);

	// yaw (y-axis rotation)
	double t2 = 2.0 * (w * y - z * x);
	if (t2 > 1.0)
	{
		t2 = 1.0;
	}
	if (t2 < -1.0)
	{
		t2 = -1.0;
	}
	double yaw = std::asin(t2);

	// roll (z-axis rotation)
	double t3 = 2.0 * (w * z + x * y);
	double t4 = 1.0 - 2.0 * (ysqr + z * z);
	double roll = std::atan2(t3, t4);

	return {roll, pitch, yaw};
}

/**
 * @brief:  位姿解算器
 *
 * @author: liqianqi
 *
 * @param:  obj: 装甲板信息，主要用四点
 *
 * @return: 装甲板在相机系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PnpSolver::poseCalculation(ArmorObject &obj)
{
	VISION_PREDICTOR_STREAM << "[pose_solver] poseCalculation" << std::endl;
	std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系
	float width_height_ratio = std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) / std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));

	VISION_PREDICTOR_STREAM << "=======================" << std::endl;
	std::vector<cv::Point2f> point_in_pixe; // 像素坐标系

	for (auto pt : obj.pts)
	{
		point_in_pixe.emplace_back(pt);
	}
	float id = obj.label;
	// point_in_pixe.push_back(obj.pts[0]);
	// point_in_pixe.push_back(obj.pts[1]);2
	// point_in_pixe.push_back(obj.pts[2]);
	// point_in_pixe.push_back(obj.pts[3]);

	VISION_PREDICTOR_STREAM << "[x,y]  " << point_in_pixe[0].x << " " << point_in_pixe[0].y << std::endl;
	VISION_PREDICTOR_STREAM << "[x,y]  " << point_in_pixe[1].x << " " << point_in_pixe[1].y << std::endl;
	VISION_PREDICTOR_STREAM << "[x,y]  " << point_in_pixe[2].x << " " << point_in_pixe[2].y << std::endl;
	VISION_PREDICTOR_STREAM << "[x,y]  " << point_in_pixe[3].x << " " << point_in_pixe[3].y << std::endl;

	if (width_height_ratio < 3.0 && (id != 1 && id != 7))
	{
		VISION_PREDICTOR_STREAM << "[notice] the small armor" << std::endl;
		is_large_ = false;
		float fHalfX = kRealSmallArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealSmallArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	}
	else
	{
		VISION_PREDICTOR_STREAM << "[notice] the large armor" << std::endl;
		is_large_ = true;
		float fHalfX = kRealLargeArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealLargeArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	}

	// VISION_PREDICTOR_STREAM << "[notice] the small armor" << std::endl;
	// float fHalfX = kRealSmallArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
	// float fHalfY = kRealSmallArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
	// point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));

	if (point_in_world.size() == 4 && point_in_pixe.size() == 4)
	{
		VISION_PREDICTOR_STREAM << "world and pixe all four points" << std::endl;
	}
	else
	{
		VISION_PREDICTOR_STREAM << "[world] size " << point_in_world.size() << std::endl;
		VISION_PREDICTOR_STREAM << "[pixe] size " << point_in_pixe.size() << std::endl;
	}

	cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

	// 世界坐标系到相机坐标系的变换
	// tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
	// rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
	// 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms
	cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs, false, cv::SOLVEPNP_SQPNP);

	cv::Mat rotM = cv::Mat::zeros(3, 3, CV_64FC1); // 解算出来的旋转矩阵

	cv::Rodrigues(rvecs, rotM);

	rotate_world_cam_ = rotM;

	Eigen::Matrix3d rotM_eigen;

	cv2eigen(rotM, rotM_eigen);
	// 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
	// Eigen::Vector3d euler_angles = rotationMatrixToEulerAngles(rotM_eigen);
	Eigen::Vector3d euler_angles = get_euler_angle(rvecs);

	// 这需要具体测量
	// double roll = euler_angles.at<double>(0);  // X-world-axis 与 X-cam-axis 在yoz上的夹角
	// double pitch = euler_angles.at<double>(1); // Y-world-axis 与 Y-cam-axis 在xoz上的夹角
	// double yaw = euler_angles.at<double>(2);   // Z-world-axis 与 Z-cam-axis 在xoy上的夹角

	double roll = euler_angles[0];	// X-world-axis 与 X-cam-axis 在yoz上的夹角   roll
	double pitch = euler_angles[1]; // Y-world-axis 与 Y-cam-axis 在xoz上的夹角   yaw
	double yaw = euler_angles[2];	// Z-world-axis 与 Z-cam-axis 在xoy上的夹角   pitch

	Eigen::Vector3d coord;
	coord << tvecs.ptr<double>(0)[0] / 100, -tvecs.ptr<double>(0)[1] / 100, tvecs.ptr<double>(0)[2] / 100;

	Eigen::Vector3d rotation;
	rotation << roll, pitch, yaw;

	VISION_PREDICTOR_STREAM << "[roll] " << roll * 180 / CV_PI << "  "
			  << "[pitch] " << pitch * 180 / CV_PI << "  "
			  << "[yaw] " << yaw * 180 / CV_PI << std::endl;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = coord;
	pose.second = rotation;

	VISION_PREDICTOR_STREAM << "[x]: " << coord[0] << "  "
			  << "[y]: " << coord[1] << "  [z]: " << coord[2] << std::endl;

	// 为什么要传相机系的旋转角，因为当无法上车调试时，可以将就着调
	VISION_PREDICTOR_STREAM << "camera pose finished!!!" << std::endl;
	VISION_PREDICTOR_STREAM << "=======================" << std::endl;
	return pose;
}

/**
 * @brief:  相机系到云台系的姿态
 *
 * @author: liqianqi
 *
 * @param:  cam_coord是相机系的坐标, rotate_world_cam是从世界系转相机系的旋转矩阵
 *
 * @return: 装甲板在云台系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PredictorPose::cam2ptz(Eigen::Vector3d &cam_coord, cv::Mat &rotate_world_cam)
{
	// cam_coord[1] = cam_coord[1] + 0.06;
	// cam_coord[2] = cam_coord[2] + 0.10;
	cam_coord[0] = cam_coord[1] + X_BIAS;
	cam_coord[1] = cam_coord[1] + Y_BIAS;
	cam_coord[2] = cam_coord[2] + Z_BIAS;

	Eigen::Matrix3d rotate_world_cam_eigen;
	cv::cv2eigen(rotate_world_cam, rotate_world_cam_eigen);

	Eigen::Matrix3d pitch_rotation_matrix_t;
	pitch_rotation_matrix_t
		<< 1,
		0, 0,
		0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
		0, -std::sin((imu_data_.pitch * CV_PI) / 180), std::cos((imu_data_.pitch * CV_PI) / 180);

	VISION_PREDICTOR_STREAM << "yaw data is " << imu_data_.yaw << std::endl;
	Eigen::Matrix3d yaw_rotation_matrix_t;
	yaw_rotation_matrix_t
		<< std::cos(imu_data_.yaw * CV_PI / 180),
		0, std::sin(imu_data_.yaw * CV_PI / 180),
		0, 1, 0,
		-std::sin(imu_data_.yaw * CV_PI / 180), 0, std::cos(imu_data_.yaw * CV_PI / 180);

	VISION_PREDICTOR_STREAM << yaw_rotation_matrix_t * pitch_rotation_matrix_t << std::endl;
	VISION_PREDICTOR_STREAM << pitch_rotation_matrix_t * yaw_rotation_matrix_t << std::endl;
	/**
	 * 写两种矩阵的原因是因为位置和姿态所用坐标系不同
	 * 用欧拉较或泰特布莱恩角表示旋转，顺序十分重要，一般是X-Y-Z，但RM这种小角度，绕定轴动轴都一样顺序什么样结果都一样
	 * 可以动手试试
	 */
	Eigen::Vector3d ptz_coord = yaw_rotation_matrix_t * pitch_rotation_matrix_t * cam_coord;
	Eigen::Matrix3d transform_vector;
	transform_vector = yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	transform_vector_ = transform_vector.inverse();

	Eigen::Vector3d rotation;
	rotation << 0, 0, 0;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = ptz_coord;
	pose.second = rotation;

	return pose;
}

Eigen::Vector3d PredictorPose::spinArmorPosition(const Eigen::Matrix<double, 11, 1> &state, double dt, int armor_group) const
{
	const double phase_offset = armor_group * (CV_PI / 2.0);
	const double theta = state[6] + state[7] * dt + phase_offset;
	const double radius = (armor_group == 0 || armor_group == 2) ? 
	                       std::max(0.0, state[8]) : std::max(0.0, state[9]);
	const double height_offset = (armor_group == 0 || armor_group == 2) ? 0.0 : state[10];
	
	Eigen::Vector3d armor;
	armor[0] = state[0] + state[1] * dt + radius * std::cos(theta);
	armor[1] = state[2] + state[3] * dt + height_offset;
	armor[2] = state[4] + state[5] * dt + radius * std::sin(theta);
	return armor;
}

Eigen::Vector3d PredictorPose::spinCenterPosition(const Eigen::Matrix<double, 11, 1> &state, double dt) const
{
	Eigen::Vector3d center;
	center[0] = state[0] + state[1] * dt;
	center[1] = state[2] + state[3] * dt;
	center[2] = state[4] + state[5] * dt;
	return center;
}

void PredictorPose::resetSpinEkf(const Eigen::Vector3d &measured_armor, double time, double r1, double r2, double delta_y)
{
	Eigen::Matrix<double, 11, 1> state = Eigen::Matrix<double, 11, 1>::Zero();
	const double use_r1 = std::clamp(r1, 0.0, kMaxSpinRadius);
	const double use_r2 = std::clamp(r2, 0.0, kMaxSpinRadius);
	
	// 假设初始跟踪的是组0装甲板 (相位=0)
	state[0] = measured_armor[0] - use_r1;  // center x
	state[1] = 0.0;                          // vx
	state[2] = measured_armor[1];            // center y
	state[3] = 0.0;                          // vy
	state[4] = measured_armor[2];            // center z
	state[5] = 0.0;                          // vz
	state[6] = 0.0;                          // phase
	state[7] = 0.0;                          // omega
	state[8] = use_r1;                       // r1 (组0/2半径)
	state[9] = use_r2 > 0 ? use_r2 : use_r1 * 1.5;  // r2 (组1/3半径，默认为r1的1.5倍)
	state[10] = delta_y;                     // 高度差
	
	spin_ekf.init(state);
	spin_ekf.P = Eigen::Matrix<double, 11, 11>::Identity() * 0.2;
	spin_ekf.P(9, 9) = 0.3;   // r2 初始不确定性更大
	spin_ekf.P(10, 10) = 0.1; // delta_y 初始不确定性
	
	current_armor_group_ = 0;
	last_location_ = measured_armor;
	last_velocity_ = Eigen::Vector3d::Zero();
	last_time_ = time;
}

void PredictorPose::initSpinFromSwitch(const Eigen::Vector3d &current_armor, const Eigen::Vector3d &last_armor, double dt)
{
	// 计算水平面上的距离来估计半径
	const double dx = current_armor[0] - last_armor[0];
	const double dz = current_armor[2] - last_armor[2];
	const double horizontal_dist = std::sqrt(dx * dx + dz * dz);
	
	// 相邻装甲板角度差为 π/2，所以 chord = 2 * r * sin(π/4) = r * sqrt(2)
	// 因此 r = chord / sqrt(2)
	const double estimated_radius = std::clamp(horizontal_dist / std::sqrt(2.0), kMinSpinRadius, kMaxSpinRadius);
	
	// 计算高度差 (相邻装甲板可能有高度差)
	const double height_diff = current_armor[1] - last_armor[1];
	
	// 估计旋转中心 (使用水平面投影)
	// 中心到两个相邻装甲板的向量夹角为 π/2
	Eigen::Vector2d v_last(last_armor[0], last_armor[2]);
	Eigen::Vector2d v_curr(current_armor[0], current_armor[2]);
	Eigen::Vector2d midpoint = (v_last + v_curr) * 0.5;
	Eigen::Vector2d perpendicular(-dz, dx);
	perpendicular.normalize();
	
	// 中心到弦中点的距离 = r * cos(π/4) = r / sqrt(2)
	const double dist_to_center = estimated_radius / std::sqrt(2.0);
	Eigen::Vector2d center_2d = midpoint + perpendicular * dist_to_center;
	
	// 判断当前装甲板属于哪组 (基于与上一个装甲板的高度差)
	// 如果高度差显著，说明切换到了另一组；否则是同组对侧
	const int new_armor_group = (std::abs(height_diff) > 0.03) ? 
	                            (1 - current_armor_group_) : current_armor_group_;
	
	// 计算相位角
	const double theta = std::atan2(current_armor[2] - center_2d[1], current_armor[0] - center_2d[0]);
	const double last_theta = std::atan2(last_armor[2] - center_2d[1], last_armor[0] - center_2d[0]);
	
	// 角速度 (相邻装甲板间隔 π/2)
	double delta_angle = normalizeAngle(theta - last_theta);
	// 确保角度变化方向正确 (应该接近 ±π/2)
	if (std::abs(delta_angle) < CV_PI / 4) {
		delta_angle = (delta_angle > 0) ? CV_PI / 2 : -CV_PI / 2;
	}
	const double omega = delta_angle / clampDt(dt);

	Eigen::Matrix<double, 11, 1> state = spin_ekf.Xe;
	state[0] = center_2d[0];
	state[1] = last_velocity_[0];
	state[2] = (current_armor_group_ == 0 || current_armor_group_ == 2) ? 
	           last_armor[1] : last_armor[1] - spin_ekf.Xe[10];  // center y (组0/2的高度)
	state[3] = last_velocity_[1];
	state[4] = center_2d[1];
	state[5] = last_velocity_[2];
	state[6] = theta - new_armor_group * (CV_PI / 2.0);  // 转换为装甲板0的相位
	state[7] = omega;
	
	// 根据当前组别更新对应的半径
	if (new_armor_group == 0 || new_armor_group == 2) {
		state[8] = estimated_radius;  // r1
	} else {
		state[9] = estimated_radius;  // r2
	}
	
	// 更新高度差
	if (std::abs(height_diff) > 0.02) {
		state[10] = (new_armor_group == 1 || new_armor_group == 3) ? 
		            height_diff : -height_diff;
	}
	
	spin_ekf.init(state);
	spin_ekf.P = Eigen::Matrix<double, 11, 11>::Identity() * 0.08;
	spin_ekf.P(9, 9) = 0.15;   // r2 不确定性
	spin_ekf.P(10, 10) = 0.05; // delta_y 不确定性

	current_armor_group_ = new_armor_group;
	left_switch_ = last_armor;
	right_switch_ = current_armor;
}

void PredictorPose::updateSpinMode(bool armor_switched, double dt)
{
	const double omega = std::abs(spin_ekf.Xe[7]);
	if (armor_switched && dt > 0.04 && dt < 1.0)
	{
		is_gyro_cnt_++;
	}
	else if (!armor_switched && is_gyro_cnt_ > 0)
	{
		is_gyro_cnt_--;
	}

	if (omega > kGyroEnterOmega || is_gyro_cnt_ >= 3)
	{
		state_ = ARMOR_STATE_::GYRO;
		is_gyro_ = true;
		return;
	}

	if (omega < kGyroExitOmega && is_gyro_cnt_ == 0)
	{
		state_ = ARMOR_STATE_::TRANSLATE;
		is_gyro_ = false;
	}
}

int PredictorPose::detectArmorGroup(const Eigen::Vector3d &armor_pos) const
{
	// 基于当前状态预测各组装甲板位置，选择最接近观测的组
	double best_dist = std::numeric_limits<double>::max();
	int best_group = current_armor_group_;
	
	for (int group = 0; group < 4; group++) {
		Eigen::Vector3d pred = spinArmorPosition(spin_ekf.Xe, 0.0, group);
		double dist = (pred - armor_pos).norm();
		if (dist < best_dist) {
			best_dist = dist;
			best_group = group;
		}
	}
	return best_group;
}

Eigen::Vector3d PredictorPose::runSpinEkf(const Eigen::Vector3d &measured_armor, double dt, bool allow_update)
{
	const double use_dt = clampDt(dt);
	
	// 设置过程噪声矩阵 Q (11x11)
	spin_ekf.Q.setZero();
	spin_ekf.Q(0, 0) = 0.01;   // center x
	spin_ekf.Q(1, 1) = 0.25;   // vx
	spin_ekf.Q(2, 2) = 0.01;   // center y
	spin_ekf.Q(3, 3) = 0.08;   // vy
	spin_ekf.Q(4, 4) = 0.01;   // center z
	spin_ekf.Q(5, 5) = 0.25;   // vz
	spin_ekf.Q(6, 6) = is_gyro_ ? 0.03 : 0.08;   // phase
	spin_ekf.Q(7, 7) = is_gyro_ ? 0.15 : 0.35;   // omega
	spin_ekf.Q(8, 8) = is_gyro_ ? 0.008 : 0.03;  // r1
	spin_ekf.Q(9, 9) = is_gyro_ ? 0.008 : 0.03;  // r2
	spin_ekf.Q(10, 10) = 0.005; // delta_y 变化较小

	spin_ekf.R.setIdentity();
	spin_ekf.R *= is_gyro_ ? 0.04 : 0.025;

	SpinPredict predict{use_dt};
	Eigen::Matrix<double, 11, 1> state = spin_ekf.predict(predict);
	
	if (allow_update)
	{
		// 检测当前观测的装甲板属于哪组
		current_armor_group_ = detectArmorGroup(measured_armor);
		
		Eigen::Matrix<double, 3, 1> measurement;
		measurement << measured_armor[0], measured_armor[1], measured_armor[2];
		SpinMeasure measure;
		measure.armor_group = current_armor_group_;
		state = spin_ekf.update(measure, measurement);
	}
	else
	{
		spin_ekf.Xe = state;
	}

	if (!state.allFinite())
	{
		resetSpinEkf(measured_armor, current_time_);
		return measured_armor;
	}

	// 约束状态变量
	spin_ekf.Xe[6] = normalizeAngle(spin_ekf.Xe[6]);
	spin_ekf.Xe[8] = std::clamp(spin_ekf.Xe[8], kMinSpinRadius, kMaxSpinRadius);  // r1
	spin_ekf.Xe[9] = std::clamp(spin_ekf.Xe[9], kMinSpinRadius, kMaxSpinRadius);  // r2
	spin_ekf.Xe[10] = std::clamp(spin_ekf.Xe[10], -0.15, 0.15);  // delta_y 约束在合理范围
	
	return spinArmorPosition(spin_ekf.Xe, 0.0, current_armor_group_);
}

/**
 * @brief:  预测主程序
 *
 * @author: liqianqi
 *
 * @param:  imu_data是IMU姿态解算的结果, objects是本帧所有的装甲板，time是图像的时间戳
 *
 * @return: 经过预测后云台上台和偏航的角度
 */
GimbalPose PredictorPose::run(GimbalPose &imu_data, std::vector<ArmorObject> &objects, double time)
{
	imu_data_ = imu_data;
	current_time_ = time;

	Eigen::Matrix3d pitch_rotation_matrix_t;
	pitch_rotation_matrix_t
		<< 1,
		0, 0,
		0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
		0, -std::sin((imu_data_.pitch * CV_PI / 180)), std::cos((imu_data_.pitch * CV_PI) / 180);

	Eigen::Matrix3d yaw_rotation_matrix_t;
	yaw_rotation_matrix_t
		<< std::cos((imu_data_.yaw * CV_PI) / 180),
		0, std::sin((imu_data_.yaw * CV_PI) / 180),
		0, 1, 0,
		-std::sin((imu_data_.yaw * CV_PI) / 180), 0, std::cos((imu_data_.yaw * CV_PI) / 180);

	Eigen::Matrix3d transform_vector;
	transform_vector =  yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	transform_vector_ = transform_vector.inverse();
	{
		const double dt = clampDt(current_time_ - last_time_);

		if (objects.empty())
		{
			loss_cnt_ = std::min(loss_cnt_ + 1, 250);
			if (init_ || state_ == ARMOR_STATE_::LOSS)
			{
				state_ = ARMOR_STATE_::LOST_REID;
				last_state_ = state_;
				return last_eular_;
			}

			if (loss_cnt_ > 30)
			{
				state_ = ARMOR_STATE_::LOST_REID;
				last_state_ = state_;
				init_ = true;
				return last_eular_;
			}

			Eigen::Vector3d filtered_armor = runSpinEkf(last_location_, dt, false);
			float fly_t = bullteFlyTime(filtered_armor);
			Eigen::Vector3d predict_location = spinArmorPosition(spin_ekf.Xe, fly_t, current_armor_group_);
			bullteFlyTime(predict_location);

			last_location_ = filtered_armor;
			last_velocity_ << spin_ekf.Xe[1], spin_ekf.Xe[3], spin_ekf.Xe[5];
			last_time_ = current_time_;
			last_state_ = state_;
			predict_location_ = predict_location;

			GimbalPose gm = gm_ptz;
			gm.yaw = gm.yaw + 1.5;
			gm.pitch = gm.pitch - 3.5;
			last_eular_ = gm;
			return gm;
		}

		loss_cnt_ = 0;
		const bool reidentify = init_ || state_ == ARMOR_STATE_::LOSS || state_ == ARMOR_STATE_::LOST_REID;
		ArmorObject obj = ArmorSelectByPrediction(objects, reidentify);
		obj_pixe_ = (obj.pts[0] + obj.pts[2]) / 2;

		if (reidentify)
		{
			VISION_PREDICTOR_STREAM << "spin ekf init finished!!!" << std::endl;
			init();
			state_ = ARMOR_STATE_::TRANSLATE;
			last_state_ = state_;
			init_ = false;
			flag = false;
			is_gyro_ = false;
			is_gyro_cnt_ = 0;
			cnt_switch = 0;
			last_obj_nums = objects.size();
			last_id = obj.label;
			last_obj = obj;
			last_pose_ = {obj.coord, obj.pyr};
			resetSpinEkf(obj.coord, current_time_);

			float fly_t = bullteFlyTime(obj.coord);
			Eigen::Vector3d predict_location = spinArmorPosition(spin_ekf.Xe, fly_t, current_armor_group_);
			bullteFlyTime(predict_location);
			predict_location_ = predict_location;

			GimbalPose gm = gm_ptz;
			gm.yaw = gm.yaw + 1.5;
			gm.pitch = gm.pitch - 3.5;
			last_eular_ = gm;
			return gm;
		}

		VISION_PREDICTOR_STREAM << "continue spin ekf track armor!!!" << std::endl;
		flag = isSwitch(obj.coord, last_obj.coord);
		if (flag)
		{
			cnt_switch++;
			initSpinFromSwitch(obj.coord, last_obj.coord, dt);
			last_switch_time_ = current_time_;
		}
		else
		{
			cnt_switch = 0;
		}

		if (std::abs(current_time_ - last_switch_time_) > 1.0)
		{
			is_gyro_cnt_ = 0;
		}

		Eigen::Vector3d filtered_armor = runSpinEkf(obj.coord, dt, true);
		updateSpinMode(flag, dt);
		time_diff = current_time_ - last_switch_time_;

		float fly_t = bullteFlyTime(filtered_armor);
		Eigen::Vector3d predict_location = spinArmorPosition(spin_ekf.Xe, fly_t, current_armor_group_);
		bullteFlyTime(predict_location);

		last_velocity_ << spin_ekf.Xe[1], spin_ekf.Xe[3], spin_ekf.Xe[5];
		move_ = (predict_location[0] - filtered_armor[0]) * 100.0;
		last_location_ = filtered_armor;
		last_time_ = current_time_;
		last_state_ = state_;
		last_pose_ = {filtered_armor, obj.pyr};
		predict_location_ = predict_location;
		last_obj = obj;
		last_id = obj.label;

		GimbalPose gm = gm_ptz;
		gm.yaw = gm.yaw + 1.5;
		gm.pitch = gm.pitch - 3.5;
		last_eular_ = gm;

		VISION_PREDICTOR_STREAM << "[spin ekf vx: " << last_velocity_[0]
		                        << " vy: " << last_velocity_[1]
		                        << " vz: " << last_velocity_[2]
		                        << " omega: " << spin_ekf.Xe[7]
		                        << " r1: " << spin_ekf.Xe[8]
		                        << " r2: " << spin_ekf.Xe[9]
		                        << " dy: " << spin_ekf.Xe[10]
		                        << " group: " << current_armor_group_ << " ]" << std::endl;
		return gm;
	}
}

/**
 * @brief  判断弹道的飞行时间
 *
 * @param  三维坐标
 *
 * @return 飞行的时间
 */
float PredictorPose::bullteFlyTime(Eigen::Vector3d coord)
{
	cv::Point3f p1;

	p1.x = coord[0];
	p1.y = coord[1];
	p1.z = coord[2];

	VISION_PREDICTOR_STREAM << "[p1.x] " << p1.x << " [p1.y] " << p1.y << " [p1.z] " << p1.z << std::endl;

	float v0_ = v0 - 0.5;

	if (v0_ == 0)
	{
		v0_ = 30;
	}

	VISION_PREDICTOR_STREAM << "bullut speed is: " << v0_ << std::endl;

	float g = 9.80665;

	// x1,y1,z1;
	// 先算yaw的值的
	float distance1;
	distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
	gm_ptz.yaw = std::asin(p1.x / distance1) * 180 / CV_PI;

	/////
	if (p1.z < 0 && p1.x < 0)
	{
		VISION_PREDICTOR_STREAM << "gm_ptz.yaw" << gm_ptz.yaw << std::endl;
		gm_ptz.yaw = gm_ptz.yaw - 2 * (gm_ptz.yaw + 90);
		VISION_PREDICTOR_STREAM << " bias " << 2 * (gm_ptz.yaw + 90) << std::endl;
	}

	if (p1.z < 0 && p1.x > 0)
	{
		gm_ptz.yaw = gm_ptz.yaw + 2 * (90 - gm_ptz.yaw);
	}

	VISION_PREDICTOR_STREAM << "[yaw: ]" << gm_ptz.yaw << std::endl;

	// pitch值
	float a = -0.5 * g * (std::pow(distance1, 2) / std::pow(v0_, 2));
	float b = distance1;
	float c = a - p1.y;

	float Discriminant = std::pow(b, 2) - 4 * a * c; // 判别式
	if (Discriminant < 0)
		return -1;
	float tan_angle1 = (-b + std::pow(Discriminant, 0.5)) / (2 * a); //*180/CV_PI;
	float tan_angle2 = (-b - std::pow(Discriminant, 0.5)) / (2 * a);

	float angle1 = std::atan(tan_angle1) * 180 / CV_PI; // 角度制
	float angle2 = std::atan(tan_angle2) * 180 / CV_PI; // 角度制

	if (tan_angle1 >= -3 && tan_angle1 <= 3)
	{
		// cout << "angle1     " << angle1 << endl;
		gm_ptz.pitch = angle1;
	}
	else if (tan_angle2 >= -3 && tan_angle2 <= 3)
	{
		// cout << "angle2     " << angle2 << endl;
		gm_ptz.pitch = angle2;
	}
	// cout << "角度是：   " << tan_angle1  << "   " << tan_angle2 << endl;
	float PI_pitch = (gm_ptz.pitch / 180) * CV_PI; // 弧度制

	VISION_PREDICTOR_STREAM << "[pitch: ]" << gm_ptz.pitch << std::endl;

	return distance1 / (v0_ * std::cos(PI_pitch));
}

ArmorObject PredictorPose::ArmorSelect(std::vector<ArmorObject> &objects)
{
	for (size_t i = 0; i < objects.size(); i++)
	{
		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
		world_cam_pose = pnp_solve_->poseCalculation(objects[i]);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
		cam_ptz_pose.first = cam3ptz(imu_data_, world_cam_pose.first);
		cam_ptz_pose.second = world_cam_pose.second;

		objects[i].coord = cam_ptz_pose.first;
		objects[i].pyr = cam_ptz_pose.second;
	}

	std::vector<float> distances(objects.size());
	for (size_t i = 0; i < objects.size(); i++)
	{
		distances[i] = std::sqrt(std::pow(objects[i].coord[0], 2) + std::pow(objects[i].coord[1], 2) + std::pow(objects[i].coord[2], 2));
	}

	float last_pose = std::sqrt(std::pow(last_pose_.first[0], 2) + std::pow(last_pose_.first[1], 2) + std::pow(last_pose_.first[2], 2));

	std::vector<float> distances_residual(objects.size());

	for (size_t i = 0; i < objects.size(); i++)
	{
		distances_residual[i] = std::abs(distances[i] - last_pose);
	}

	size_t index = 0;
	for (size_t i = 1; i < objects.size(); i++)
	{
		if (distances_residual[i] < distances_residual[index])
			index = i;
	}

	return objects[index];
}

ArmorObject PredictorPose::ArmorSelectByPrediction(std::vector<ArmorObject> &objects, bool reidentify)
{
	for (size_t i = 0; i < objects.size(); i++)
	{
		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose = pnp_solve_->poseCalculation(objects[i]);
		objects[i].coord = cam3ptz(imu_data_, world_cam_pose.first);
		objects[i].pyr = world_cam_pose.second;
	}

	if (objects.empty())
	{
		return ArmorObject();
	}

	Eigen::Vector3d reference = last_location_;
	if (!init_ && state_ != ARMOR_STATE_::LOSS)
	{
		reference = spinArmorPosition(spin_ekf.Xe, clampDt(current_time_ - last_time_), current_armor_group_);
	}

	size_t index = 0;
	double best_score = std::numeric_limits<double>::max();
	for (size_t i = 0; i < objects.size(); i++)
	{
		double score = (objects[i].coord - reference).norm();
		if (!reidentify && objects[i].label != last_id)
		{
			score += 0.4;
		}
		if (reidentify && score > kLostReidDistanceGate)
		{
			score += 0.8;
		}
		if (score < best_score)
		{
			best_score = score;
			index = i;
		}
	}

	return objects[index];
}

bool PredictorPose::isSwitch(Vector3d up_switch, Vector3d down_switch)
{
	float distance_;
	distance_ = std::sqrt(up_switch[0] * up_switch[0] + up_switch[2] * up_switch[2]);
	float yaw_now = std::asin(up_switch[0] / distance_) * 180 / CV_PI;

	float distance_last;
	distance_last = std::sqrt(down_switch[0] * down_switch[0] + down_switch[2] * down_switch[2]);
	float yaw_last = std::asin(down_switch[0] / distance_last) * 180 / CV_PI;
	if (std::abs(yaw_now - yaw_last) > 3.0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

Eigen::Vector3d PredictorPose::cam3ptz(GimbalPose gm, Vector3d pos)
{
	pos[0] = pos[0] + X_BIAS;
	pos[1] = pos[1] + Y_BIAS;
	pos[2] = pos[2] + Z_BIAS;
	// pos = pos.transpose();

	Matrix3d pitch_rotation_matrix_;
	Matrix3d yaw_rotation_matrix_;

	VISION_PREDICTOR_STREAM << "camera pose " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
	VISION_PREDICTOR_STREAM << "solve gm.pitch" << gm.pitch << std::endl;
	VISION_PREDICTOR_STREAM << "solve gm.yaw" << gm.yaw << std::endl;

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

	Vector3d t_pos_ =  yaw_rotation_matrix_ * pitch_rotation_matrix_ * pos;

	VISION_PREDICTOR_STREAM << "world pose " << t_pos_[0] << " " << t_pos_[1] << " " << t_pos_[2] << std::endl;

	return t_pos_;
}
