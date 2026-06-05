#include "../../include/thread/thread.h"
// #define SAVE_VIDEO
// #define RECORD_DATA
#define MIDVISION
// #define VIDEO

boost::system::error_code ec;
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev);

// 世界坐标系内坐标--->相机坐标系内坐标
inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_CW)
{
	Eigen::Vector3d pw_t;
	pw_t = R_CW * pw;
	pw_t[0] = pw_t[0] - X_BIAS;
	pw_t[1] = pw_t[1] - Y_BIAS;
	pw_t[2] = pw_t[2] - Z_BIAS;

	pw_t[1] = -pw_t[1];

	return pw_t;
}

// 相机坐标系内坐标--->图像坐标系内像素坐标
inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc, const Eigen::Matrix3d &F)
{
	return F * pc / pc(2, 0);
}

// 将世界坐标系内一点，投影到图像中，并绘制该点
// inline void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw,
// 							 const Eigen::Matrix3d &R_IW, const cv::Scalar &color)
// {
// 	Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
// 	Eigen::Vector3d pu = pc_to_pu(pc);
// 	cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
// }

namespace MidCamera
{
	int MV_exp_value = 6621;
	MVCamera *camera_ptr_ = nullptr;
	void MVSetExpTime(int, void *)
	{
		camera_ptr_->SetExpose(MV_exp_value);
	}
}

void Factory::producer()
{
#ifdef MIDVISION
	while (true)
	{
		cv::Mat frame;
		double timestamp = 0;
		if (captureFrame(frame, timestamp))
		{
			pushFrame(frame, timestamp);
		}
	}
#endif
}

void Factory::consumer()
{
	while (true)
	{
		cv::Mat img;
		double src_time = 0;
		if (!popLatestFrame(img, src_time))
		{
			std::this_thread::yield();
			continue;
		}
#if VISION_ENABLE_LOG
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
		processFrame(img, src_time);
		sendVisionData();
		drawDebugOverlay(img);
#if VISION_ENABLE_LOG
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
		VISION_LOG("                                " << "帧率：" << 1 / time_run.count() << std::endl);
#endif
	}
}

Horizon::DataControler::Stm32Data Factory::TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time)
{
	VISION_LOG("stm32s size() " << stm32s.size() << std::endl);
	if (stm32s.size() == 0)
	{
		Horizon::DataControler::Stm32Data a;
		return a;
	}
	int index = 0;

	// for(auto stm32 : stm32s)
	// {
	// 	if(!stm32.dubug_print)
	// 	{
	// 		// std::cout << "true" << std::endl;
	// 	}
	// }

	vector<double> scale_time;
	scale_time.reserve(1000);

	for (int i = 0; i < stm32s.size(); i++)
	{
		scale_time.push_back(src_time - stm32s[i].time.f);
	}

	for (int i = 0; i < stm32s.size(); i++)
	{
		if (std::abs(scale_time[i]) < std::abs(scale_time[index]))
		{
			index = i;
		}
	}
	VISION_LOG("finished!!" << std::endl);
	Horizon::DataControler::Stm32Data stm32 = stm32s[index];

	// stm32data.dubug_print = stm32s[index].dubug_print;
	// stm32data.pitch_data_.f = stm32s[index].pitch_data_.f;
	// stm32data.yaw_data_.f = stm32s[index].yaw_data_.f;
	// stm32data.time.f = stm32s[index].time.f;
	// stm32data.init_firing_rate = stm32s[index].init_firing_rate;

	stm32data = stm32;

	return stm32;
}

void Factory::getdata()
{
	openSerial();
	while (1)
	{
		receiveSerialFrame();
	}
}