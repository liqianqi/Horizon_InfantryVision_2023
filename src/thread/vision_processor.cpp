#include "../../include/thread/thread.h"

void Factory::syncImuData(double src_time)
{
	std::lock_guard<std::mutex> lock(serial_mutex_);
	TimeSynchronization(MCU_data_, src_time);
	imu_data.pitch = stm32data.pitch_data_.f;
	imu_data.yaw = stm32data.yaw_data_.f;
	imu_data.timestamp = stm32data.time.f;

	predic_pose_->v0 = stm32data.init_firing_rate;
	predic_pose_->v0 = 27;

	if (stm32data.color_)
	{
		infer.set_color_ = COLOR::BLUE;
	}
	else
	{
		infer.set_color_ = COLOR::RED;
	}
}

void Factory::processFrame(cv::Mat &img, double src_time)
{
	VISION_LOG("time :" << src_time << std::endl);
	syncImuData(src_time);
	VISION_LOG("stm32 speed is: " << stm32data.init_firing_rate << std::endl);

	if (stm32data.flag == 0)
	{
		VISION_LOG("自瞄" << std::endl);
		std::vector<ArmorObject> objects = infer.run(img);
		gim = predic_pose_->run(imu_data, objects, src_time);
		coord = predic_pose_->last_pose_.first;
		rotation = predic_pose_->last_pose_.second;

		if (objects.empty())
		{
			visiondata.is_have_armor = false;
		}
		else
		{
			visiondata.yaw_data_.f = gim.yaw;
			visiondata.pitch_data_.f = gim.pitch;
			visiondata.time.f = src_time;
			visiondata.is_have_armor = true;
		}
	}
	else
	{
		VISION_LOG("打符" << std::endl);
		std::vector<BUFF::BuffObject> objs = buff_infer.run(img);
		gim = buffpredictor.run(src_time, objs, imu_data);
		coord = buffpredictor.last_coord_ptz_.first;
		rotation = buffpredictor.last_coord_ptz_.second;

		if (objs.empty())
		{
			visiondata.is_have_armor = false;
		}
		else
		{
			visiondata.yaw_data_.f = gim.yaw;
			visiondata.pitch_data_.f = gim.pitch;
			visiondata.time.f = src_time;
			visiondata.is_have_armor = true;
		}
	}
}

void Factory::sendVisionData()
{
	std::lock_guard<std::mutex> lock(serial_mutex_);
	data_controler_.sentData(fd, visiondata);
}
