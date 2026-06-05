#include "../../include/thread/thread.h"

namespace MidCamera
{
	extern int MV_exp_value;
	extern MVCamera *camera_ptr_;
	void MVSetExpTime(int, void *);
}

void Factory::initCamera()
{
	MidCamera::camera_ptr_ = new MVCamera;

#if VISION_ENABLE_CAMERA_TUNING_UI
	cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
	cv::createTrackbar("MVExpTime", "MVCameraDebug", &MidCamera::MV_exp_value, 15000, MidCamera::MVSetExpTime);
#endif

	std::lock_guard<std::mutex> lock(image_mutex_);
	image_buffer_front_ = 0;
	image_buffer_rear_ = 0;
}

bool Factory::captureFrame(cv::Mat &frame, double &timestamp)
{
	static std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

	if (MidCamera::camera_ptr_ == nullptr)
	{
		initCamera();
		return false;
	}

	VISION_LOG("enter producer" << std::endl);
	while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER - 1)
	{
		std::this_thread::yield();
	}

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	if (!MidCamera::camera_ptr_->GetMat(frame))
	{
		delete MidCamera::camera_ptr_;
		MidCamera::camera_ptr_ = nullptr;
		return false;
	}

	std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
	timestamp = time_run.count();
	MidCamera::camera_ptr_->SetExpose(MidCamera::MV_exp_value);
	return true;
}

void Factory::pushFrame(const cv::Mat &frame, double timestamp)
{
	std::lock_guard<std::mutex> lock(image_mutex_);
	image_buffer_[image_buffer_front_ % IMGAE_BUFFER] = frame;
	timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = timestamp;
	++image_buffer_front_;
}

bool Factory::popLatestFrame(cv::Mat &frame, double &timestamp)
{
	std::lock_guard<std::mutex> lock(image_mutex_);
	if (image_buffer_front_ <= image_buffer_rear_)
	{
		return false;
	}

	image_buffer_rear_ = image_buffer_front_ - 1;
	frame = image_buffer_[image_buffer_rear_ % IMGAE_BUFFER].clone();
	timestamp = timer_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
	return true;
}
