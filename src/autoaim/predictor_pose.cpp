#include "../../include/autoaim/predictor_pose.h"

PnpSolver::PnpSolver(const string yaml)
{
	cv::FileStorage fs(yaml, cv::FileStorage::READ);
	fs["M1"] >> K_;
	fs["D1"] >> distCoeffs_;
	fs.release();
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
	std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系
	float width_height_ratio = std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) / std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));

	std::vector<cv::Point2f> point_in_pixe; // 像素坐标系

	for (auto pt : obj.pts)
	{
		point_in_pixe.emplace_back(pt);
	}

	if (width_height_ratio > 0.35 && width_height_ratio < 0.55)
	{
		float fHalfX = kRealSmallArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealSmallArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	}
	else if (width_height_ratio > 0.20 && width_height_ratio < 0.30)
	{
		float fHalfX = kRealLargeArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealLargeArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	}

	cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

	// 世界坐标系到相机坐标系的变换
	// tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
	// rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
	// 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms
	cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs);

	cv::Mat rotM = cv::Mat::zeros(3, 3, CV_64FC1); // 解算出来的旋转矩阵

	cv::Rodrigues(rvecs, rotM);

	rotate_world_cam_ = rotM;

	// 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
	cv::Mat rot_mat_r, rot_mat_q;
	cv::RQDecomp3x3(rotM, rot_mat_r, rot_mat_q);
	cv::Mat euler_angles = -rot_mat_r.t() * rot_mat_q.t();

	// 这需要具体测量
	double roll = euler_angles.at<double>(0);  // X-world-axis 与 X-cam-axis 在yoz上的夹角
	double pitch = euler_angles.at<double>(1); // Y-world-axis 与 Y-cam-axis 在xoz上的夹角
	double yaw = euler_angles.at<double>(2);   // Z-world-axis 与 Z-cam-axis 在xoy上的夹角

	Eigen::Vector3d coord;
	coord << tvecs.ptr<double>(0)[0], -tvecs.ptr<double>(0)[1], tvecs.ptr<double>(0)[2];

	Eigen::Vector3d rotation;
	rotation << roll, pitch, yaw;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = coord;
	pose.second = rotation;

	// 为什么要传相机系的旋转角，因为当无法上车调试时，可以将就着调
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
std::pair<Eigen::Vector3d,Eigen::Vector3d> PredictorPose::cam2ptz(Eigen::Vector3d &cam_coord, cv::Mat &rotate_world_cam)
{
	Eigen::Matrix3d rotate_world_cam_eigen;
	cv::cv2eigen(rotate_world_cam,rotate_world_cam_eigen);
	
	Eigen::Matrix3d yaw_rotation_matrix_R;
	yaw_rotation_matrix_R
    <<
    std::cos(imu_data_.yaw),  0,  std::sin(imu_data_.yaw),
    		0,                1,            0,
    -std::sin(imu_data_.yaw), 0,  std::cos(imu_data_.yaw);

	Eigen::Matrix3d pitch_rotation_matrix_R;
	pitch_rotation_matrix_R
    <<
    1,              0,              			0,
    0,  std::cos(imu_data_.pitch),  -std::sin(imu_data_.pitch),
    0,  std::sin(imu_data_.pitch),  std::cos(imu_data_.pitch);

    Eigen::Matrix3f pitch_rotation_matrix_t;
    pitch_rotation_matrix_t
    <<
    1,              0,              		0,
    0,  std::cos(imu_data_.pitch),  std::sin(imu_data_.pitch),
    0,  -std::sin(imu_data_.pitch), std::cos(imu_data_.pitch);

	Eigen::Matrix3f yaw_rotation_matrix_t;
    yaw_rotation_matrix_t
    <<
    std::cos(imu_data_.yaw),  0,  std::sin(imu_data_.yaw),
    0,                 		  1,            0,
    -std::sin(imu_data_.yaw), 0,  std::cos(imu_data_.yaw);

	/**
	 * 写两种矩阵的原因是因为位置和姿态所用坐标系不同
	 * 用欧拉较或泰特布莱恩角表示旋转，顺序十分重要，一般是X-Y-Z，但RM这种小角度，绕定轴动轴都一样顺序什么样结果都一样
	 * 可以动手试试
	*/
	Eigen::Vector3f ptz_coord = pitch_rotation_matrix_t * yaw_rotation_matrix_t * cam_coord;

	Eigen::Matrix3d rotate_cam_ptz_eigen = pitch_rotation_matrix_R*yaw_rotation_matrix_R*rotate_world_cam_eigen;

	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::eigen2cv(rotate_cam_ptz_eigen,R);

	// 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
	cv::Mat rot_mat_r, rot_mat_q;
	cv::RQDecomp3x3(R, rot_mat_r, rot_mat_q);
	cv::Mat euler_angles = -rot_mat_r.t() * rot_mat_q.t();

	// 这需要具体测量
	double roll = euler_angles.at<double>(0);  // X-world-axis 与 X-cam-axis 在yoz上的夹角
	double pitch = euler_angles.at<double>(1); // Y-world-axis 与 Y-cam-axis 在xoz上的夹角
	double yaw = euler_angles.at<double>(2);   // Z-world-axis 与 Z-cam-axis 在xoy上的夹角

	Eigen::Vector3d rotation;
	rotation << roll, pitch, yaw;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = ptz_coord;
	pose.second = rotation;

	return pose;
}

GimbalPose PredictorPose::run(GimbalPose &imu_data,std::vector<ArmorObject> &objects,double time)
{
	imu_data_ = imu_data;
	current_time_ = time;
	// 如果这一帧没有装甲板，先看看init_是true还是false
	// 是true

	if(!objects.size())
	{
		loss_cnt_++;
		if(loss_cnt_ > 200)
		{
			loss_cnt_ = 200;
		}
		if(loss_cnt_ > 100 || last_state_ == ARMOR_STATE_::LOSS)
		{
			state_ = ARMOR_STATE_::LOSS;
			last_state_ = state_;
			// 返回最后一帧的云台角，避免剧烈晃动
			// return
			init_ = true;
			float fly_t = bullteFlyTime(last_pose_.first);
			GimbalPose gm = gm_ptz;
			return gm;
		}
		state_ = ARMOR_STATE_::TRACK;
		last_state_ = state_;
		// 根据上一帧位置和速度在做预测，100次  !!! return
		Eigen::Vector3d current_pose;
		current_pose[0] = last_location_[0] + last_velocity_[0]*(current_time_ - last_time_);
		current_pose[1] = last_location_[1] + last_velocity_[1]*(current_time_ - last_time_);
		current_pose[2] = last_location_[2] + last_velocity_[2]*(current_time_ - last_time_);
		float fly_t = bullteFlyTime(current_pose);

		Eigen::Vector3d predict_pose;
		predict_pose[0] = current_pose[0]*fly_t;
		predict_pose[1] = current_pose[1]*fly_t;
		predict_pose[2] = current_pose[2]*fly_t;

		bullteFlyTime(current_pose);
		last_time_ = current_time_;
		last_location_ = current_pose;
		GimbalPose gm = gm_ptz;
		return gm;
	}

	loss_cnt_ = 0;
	if(init_)
	{
		init();
		state_ = ARMOR_STATE_::INIT;
		last_state_ = state_;
		// 选择一个装甲板
		ArmorObject obj = ArmorSelect(objects);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
		world_cam_pose = pnp_solve_->poseCalculation(obj);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
		cam_ptz_pose = cam2ptz(cam_ptz_pose.first,pnp_solve_->rotate_world_cam_);

		last_pose_ = cam_ptz_pose;		// 记录这一时刻，为下一时刻预测做准备
		init_ = false;
		// !!! return 
		float fly_t = bullteFlyTime(cam_ptz_pose.first);
		last_time_ = current_time_;
		GimbalPose gm = gm_ptz;
		return gm;
	}

	/**
	 * 陀螺模块相对复杂，要测试的东西多，开学再写
	*/

	// 选择一个装甲板
	// 当卡方检验过大时仍然要打开初始化开关
	state_ = ARMOR_STATE_::TRACK;
	ArmorObject obj = ArmorSelect(objects);

	std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
	world_cam_pose = pnp_solve_->poseCalculation(obj);

	std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
	cam_ptz_pose = cam2ptz(cam_ptz_pose.first,pnp_solve_->rotate_world_cam_);

	float fly_t = bullteFlyTime(cam_ptz_pose.first);

	Eigen::Vector3d current_v;	// 现在的速度: 直接计算的速度，未经过拟合
	current_v[0] = (cam_ptz_pose.first[0] - last_pose_.first[0])/(current_time_-last_time_);
	current_v[1] = (cam_ptz_pose.first[1] - last_pose_.first[1])/(current_time_-last_time_);
	current_v[2] = (cam_ptz_pose.first[2] - last_pose_.first[2])/(current_time_-last_time_);
	
	velocities_.Enqueue(current_v);

	Eigen::Vector3d now_v = CeresVelocity(velocities_);

	Eigen::Vector3d predict_location;
	predict_location[0] = cam_ptz_pose.first[0] + (now_v[0]*fly_t);
	predict_location[1] = cam_ptz_pose.first[1] + (now_v[1]*fly_t);
	predict_location[2] = cam_ptz_pose.first[2] + (now_v[2]*fly_t);

	bullteFlyTime(predict_location);

	last_location_ = cam_ptz_pose.first;
	last_time_ = current_time_;
	last_velocity_ = now_v;
	last_state_ = state_;
	last_pose_ = cam_ptz_pose;
		
	GimbalPose gm = gm_ptz;
	return gm;
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

    float v0_ = v0;

    float g = 9.80665;

    // x1,y1,z1;
    // 先算yaw的值的
    float distance1;
    distance1 = std::sqrt(p1.x*p1.x+p1.z*p1.z);
    gm_ptz.yaw = std::sin(p1.x/distance1)*180/CV_PI;

    // pitch值
    float a = -0.5*g*(std::pow(distance1,2)/std::pow(v0_,2));
    float b = distance1;
    float c = a - p1.y;
    
    float Discriminant = std::pow(b,2) - 4*a*c;  // 判别式
    if(Discriminant < 0) return -1;
    float tan_angle1 = (-b + std::pow(Discriminant,0.5))/(2*a); //*180/CV_PI;
    float tan_angle2 = (-b - std::pow(Discriminant,0.5))/(2*a);

    float angle1 = std::atan(tan_angle1)*180/CV_PI;	// 角度制
    float angle2 = std::atan(tan_angle2)*180/CV_PI; // 角度制

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
    float PI_pitch = (gm_ptz.pitch/180)*CV_PI; // 弧度制

    return distance1/(v0_*std::cos(PI_pitch));

}