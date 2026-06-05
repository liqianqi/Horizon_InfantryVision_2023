#include "../../include/thread/thread.h"

namespace
{
#if VISION_ENABLE_DEBUG_UI
inline Eigen::Vector3d pw_to_pc_debug(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_CW)
{
	Eigen::Vector3d pw_t = R_CW * pw;
	pw_t[0] = pw_t[0] - X_BIAS;
	pw_t[1] = pw_t[1] - Y_BIAS;
	pw_t[2] = pw_t[2] - Z_BIAS;
	pw_t[1] = -pw_t[1];
	return pw_t;
}

inline Eigen::Vector3d pc_to_pu_debug(const Eigen::Vector3d &pc, const Eigen::Matrix3d &F)
{
	return F * pc / pc(2, 0);
}
#endif
}

void Factory::drawDebugOverlay(cv::Mat &img)
{
#if VISION_ENABLE_DEBUG_UI
	char test[100];
	sprintf(test, "x:%0.4f", coord[0]);
	cv::putText(img, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "y:%0.4f", coord[1]);
	cv::putText(img, test, cv::Point(img.cols / 3, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "z:%0.4f", coord[2]);
	cv::putText(img, test, cv::Point(2 * img.cols / 3, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "roll:%0.4f", rotation[0] * 180 / CV_PI);
	cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "pitch:%0.4f", rotation[1] * 180 / CV_PI);
	cv::putText(img, test, cv::Point(img.cols / 3, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "yaw:%0.4f", rotation[2] * 180 / CV_PI);
	cv::putText(img, test, cv::Point(2 * img.cols / 3, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "get yaw:%0.4f ", stm32data.yaw_data_.f);
	cv::putText(img, test, cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "get pitch:%0.4f ", stm32data.pitch_data_.f);
	cv::putText(img, test, cv::Point(img.cols / 2, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_.f);
	cv::putText(img, test, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_.f);
	cv::putText(img, test, cv::Point(img.cols / 2, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, " is_get:%s ", stm32data.dubug_print ? "true" : "false");
	cv::putText(img, test, cv::Point(10, 420), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, " armor size:%s ", predic_pose_->pnp_solve_->is_large_ ? "large" : "small");
	cv::putText(img, test, cv::Point(img.cols / 2, 540), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	sprintf(test, " is have:%s ", visiondata.is_have_armor ? "true" : "false");
	cv::putText(img, test, cv::Point(img.cols / 2, 580), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

	if (stm32data.flag == 0)
	{
		if (predic_pose_->flag)
		{
			cv::circle(img, {int(30), int(800)}, 3, cv::Scalar(0, 0, 255), cv::LINE_AA);
		}
		Eigen::Vector3d pc = pw_to_pc_debug(predic_pose_->predict_location_, predic_pose_->transform_vector_);
		Eigen::Matrix3d F;
		cv2eigen(predic_pose_->pnp_solve_->K_, F);
		Eigen::Vector3d pu = pc_to_pu_debug(pc, F);
		cv::circle(img, {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 165, 255), cv::LINE_AA);

		if (predic_pose_->is_gyro_)
		{
			cv::circle(img, {int(60), int(900)}, 6, cv::Scalar(128, 0, 128), cv::LINE_AA);

			Eigen::Vector3d pc1 = pw_to_pc_debug(predic_pose_->left_switch_, predic_pose_->transform_vector_);
			Eigen::Matrix3d F1;
			cv2eigen(predic_pose_->pnp_solve_->K_, F1);
			Eigen::Vector3d pu1 = pc_to_pu_debug(pc1, F1);
			cv::circle(img, {int(pu1(0, 0)), int(pu1(1, 0) - 70)}, 3, cv::Scalar(255, 0, 0), cv::LINE_AA);

			Eigen::Vector3d pc2 = pw_to_pc_debug(predic_pose_->right_switch_, predic_pose_->transform_vector_);
			Eigen::Matrix3d F2;
			cv2eigen(predic_pose_->pnp_solve_->K_, F2);
			Eigen::Vector3d pu2 = pc_to_pu_debug(pc2, F2);
			cv::circle(img, {int(pu2(0, 0)), int(pu2(1, 0) - 70)}, 3, cv::Scalar(255, 255, 255), cv::LINE_AA);
		}
	}
	else if (stm32data.flag == 1)
	{
		Eigen::Vector3d pc = pw_to_pc_debug(buffpredictor.last_coord_ptz_.first, buffpredictor.transform_vector_);
		Eigen::Matrix3d F;
		cv2eigen(predic_pose_->pnp_solve_->K_, F);
		Eigen::Vector3d pu = pc_to_pu_debug(pc, F);
		cv::circle(img, {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 165, 255), cv::LINE_AA);

		Eigen::Vector3d pc2 = pw_to_pc_debug(buffpredictor.R_use, buffpredictor.transform_vector_);
		Eigen::Matrix3d F2;
		cv2eigen(predic_pose_->pnp_solve_->K_, F2);
		Eigen::Vector3d pu2 = pc_to_pu_debug(pc2, F2);
		cv::circle(img, {int(pu2(0, 0)), int(pu2(1, 0))}, 3, cv::Scalar(255, 255, 255), cv::LINE_AA);
	}

	std::string windowName = "show";
	cv::namedWindow(windowName, 0);
	cv::imshow(windowName, img);
	cv::waitKey(1);
#else
	(void)img;
#endif
}
