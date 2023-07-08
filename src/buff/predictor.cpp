#include "../../include/buff/predictor.h"

namespace BUFF
{

PnpSolver_::PnpSolver_(const string yaml)
{
	cv::FileStorage fs(yaml, cv::FileStorage::READ);
	fs["M1"] >> K_;
	fs["D1"] >> distCoeffs_;
	fs.release();
}

Eigen::Vector3d PnpSolver_::get_euler_angle(cv::Mat rotation_vector)
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

	return {roll, yaw, pitch};
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
std::pair<Eigen::Vector3d, Eigen::Vector3d> PnpSolver_::poseCalculation(BuffObject &obj)
{
	std::cout << "[pose_solver] poseCalculation" << std::endl;
	std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系
	// float width_height_ratio = std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) / std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));

	std::cout << "=======================" << std::endl;
	std::vector<cv::Point2f> point_in_pixe; // 像素坐标系

	// for (auto pt : obj.pts)
	// {
	// 	std::cout << "&&&&&&&&&&&" << std::endl;
	// 	point_in_pixe.emplace_back(pt);
	// }
	point_in_pixe.emplace_back(obj.pts[0]);
	point_in_pixe.emplace_back(obj.pts[1]);
	point_in_pixe.emplace_back(obj.pts[2]);
	point_in_pixe.emplace_back(obj.pts[4]);


	std::cout << "[notice] the rune armor" << std::endl;
	float fHalfX = RealSmallArmorWidth;	 // 将装甲板的宽的一半作为原点的x
	float fHalfY = RealSmallArmorHeight; // 将装甲板的宽的一半作为原点的y

	point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
	point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
	point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
	//point_in_world.emplace_back(cv::Point3f(0, -60, 0));
	point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));

	if (point_in_world.size() == 4 && point_in_pixe.size() == 4)
	{
		std::cout << "world and pixe all four points" << std::endl;
	}
	else
	{
		std::cout << "[world] size " << point_in_world.size() << std::endl;
		std::cout << "[pixe] size " << point_in_pixe.size() << std::endl;
	}

	cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

	// 世界坐标系到相机坐标系的变换
	// tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
	// rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
	// 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms
	cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs, cv::SOLVEPNP_ITERATIVE);

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
	double yaw = euler_angles[1];	// Y-world-axis 与 Y-cam-axis 在xoz上的夹角   yaw
	double pitch = euler_angles[2]; // Z-world-axis 与 Z-cam-axis 在xoy上的夹角   pitch

	Eigen::Vector3d coord;
	coord << tvecs.ptr<double>(0)[0] / 100, -tvecs.ptr<double>(0)[1] / 100, tvecs.ptr<double>(0)[2] / 100;

	Eigen::Vector3d rotation;
	rotation << roll, pitch, yaw;

	std::cout << "[roll] " << roll * 180 / CV_PI << "  "
			  << "[pitch] " << pitch * 180 / CV_PI << "  "
			  << "[yaw] " << yaw * 180 / CV_PI << std::endl;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = coord;
	pose.second = rotation;

	std::cout << "[x]: " << coord[0] << "  "
			  << "[y]: " << coord[1] << "  [z]: " << coord[2] << std::endl;

	// 为什么要传相机系的旋转角，因为当无法上车调试时，可以将就着调
	std::cout << "camera pose finished!!!" << std::endl;
	std::cout << "=======================" << std::endl;
	return pose;
}

GimbalPose BuffPredictor::run(double src_time,std::vector<BuffObject> objects, GimbalPose imu, int flag)
{
	if(objects.size() == 0)
	{
		return gm_ptz;
	}

	// imu_data_ = imu_data;
	// current_time_ = time;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> coord_cam = pnp->poseCalculation(objects[0]);
	std::pair<Eigen::Vector3d, Eigen::Vector3d> coord_ptz;
	coord_ptz.first = cam3ptz(imu,coord_cam.first);
	coord_ptz.second = coord_cam.second;
	Eigen::Vector3d R = getR(coord_ptz.first, coord_ptz.second[0]);

	if(std::abs(coord_ptz.second[0] - last_coord_ptz_.second[0])*180/CV_PI > 30)
	{
		rolls.clear();
	}

	// if (R_QUENE.size() < r_size)
	// {
	// 	R_QUENE.push_back(R);
	// }
	// else
	// {
	// 	R_QUENE.pop_front();
	// 	R_QUENE.push_back(R);
	// }

	//R_use = slide_windows_R(R_QUENE);
	R_use = R;

	if(rolls.size() < roll_size)
	{
		Eigen::Vector2d roll_time;
		roll_time[0] = coord_ptz.second[0];
		roll_time[1] = src_time;
		rolls.push_back(roll_time);
	}
	else
	{
		rolls.pop_front();
		Eigen::Vector2d roll_time;
		roll_time[0] = coord_ptz.second[0];
		roll_time[1] = src_time;
		rolls.push_back(roll_time);
	}
	double now_rad_v = CeresRads(rolls);

	float t = bullteFlyTime(coord_ptz.first);

	Eigen::Vector3d location_point;
	double roll_predict;
	roll_predict = coord_ptz.second[0] + (t+0.0)*now_rad_v;

	location_point[0] = R_use[0] + 0.7*std::sin(roll_predict);
	location_point[1] = R_use[1] + 0.7*std::cos(roll_predict);
	location_point[2] = R_use[2];

	float t0 = bullteFlyTime(location_point);

	last_coord_ptz_ = coord_ptz;
	
	gm_ptz.yaw = gm_ptz.yaw + 2.0;
	gm_ptz.pitch = gm_ptz.pitch - 3.0;

	return gm_ptz;
}

Eigen::Vector3d BuffPredictor::getR(Eigen::Vector3d coord, double roll) // rad
{
	Eigen::Vector3d R;
	R[0] = coord[0] - 0.7*std::sin(roll);
	R[1] = coord[1] - 0.7*std::cos(roll);
	R[2] = coord[2];
	return R;
}

double BuffPredictor::CeresRads(std::deque<Eigen::Vector2d> rolls)
{
	int N = rolls.size();
	if (rolls.size() == 0 || rolls.size() == 1)
	{
		return 0;
	}

	double avg_x = 0;
	double avg_x2 = 0;
	double avg_f = 0;
	double avg_xf = 0;

	double time_first = rolls.front()[1];

	for (int i = 0; i < N; i++)
	{
		avg_x += rolls[i][1] - time_first;
		avg_x2 += std::pow(rolls[i][1] - time_first, 2);
		avg_f += rolls[i][0];
		avg_xf += (rolls[i][1] - time_first) * rolls[i][0];
	}
	double v0 = (avg_xf - N * (avg_x / N) * (avg_f / N)) / (avg_x2 - N * std::pow(avg_x / N, 2));

	return v0;
}

/**
 * @brief  判断弹道的飞行时间
 *
 * @param  三维坐标
 *
 * @return 飞行的时间
 */
float BuffPredictor::bullteFlyTime(Eigen::Vector3d coord)
{
	cv::Point3f p1;

	p1.x = coord[0];
	p1.y = coord[1];
	p1.z = coord[2];

	std::cout << "[p1.x] " << p1.x << " [p1.x] " << p1.y << " [p1.z] " << p1.z << std::endl;

	float v0_ = 28;

	float g = 9.80665;

	// x1,y1,z1;
	// 先算yaw的值的
	float distance1;
	distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
	gm_ptz.yaw = std::asin(p1.x / distance1) * 180 / CV_PI;

	if (p1.z < 0 && p1.x < 0)
	{
		std::cout << "gm_ptz.yaw" << gm_ptz.yaw << std::endl;
		gm_ptz.yaw = gm_ptz.yaw - 2 * (gm_ptz.yaw + 90);
		std::cout << " bias " << 2 * (gm_ptz.yaw + 90) << std::endl;
	}

	if (p1.z < 0 && p1.x > 0)
	{
		gm_ptz.yaw = gm_ptz.yaw + 2 * (90 - gm_ptz.yaw);
	}
	// std::cout << "[yaw: ]" << gm_ptz.yaw << std::endl;

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

	// std::cout << "[pitch: ]" << gm_ptz.pitch << std::endl;

	return distance1 / (v0_ * std::cos(PI_pitch));
}

}