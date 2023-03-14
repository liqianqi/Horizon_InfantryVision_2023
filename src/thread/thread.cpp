#include "../../include/thread/thread.h"
// #define SAVE_VIDEO
// #define RECORD_DATA
#define DAHENG
//#define MIDVISION
//#define VIDEO
mutex image_mutex_{}; // 数据上🔓

namespace GxCamera
{
	// int GX_exp_time = 2533;

	// int GX_gain = 4;
	int GX_exp_time = 3133;

	int GX_gain = 0;
	DaHengCamera *camera_ptr_ = nullptr;
	int GX_blance_r = 50;
	int GX_blance_g = 32;
	int GX_blance_b = 44;

	int GX_gamma = 1;

	// DaHengCamera* camera_ptr_ = nullptr;

	void DaHengSetExpTime(int, void *)
	{
		camera_ptr_->SetExposureTime(GX_exp_time);
	}

	void DaHengSetGain(int, void *)
	{
		camera_ptr_->SetGain(3, GX_gain);
	}

}

namespace MidCamera
{
	int MV_exp_value = 2500;
	MVCamera *camera_ptr_ = nullptr;
	void MVSetExpTime(int, void *)
	{
		camera_ptr_->SetExpose(MV_exp_value);
	}
}

void Factory::producer()
{

#ifdef VIDEO
	cv::VideoCapture cap("/home/liqianqi/Horizon_InfantryVision-2023/src/thread/blue_buff.mp4");
	cv::Mat src;
#ifdef SAVE_VIDEO
	// cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	cap >> src;
	std::cout << src.size().width << "   " << src.size().height << std::endl;
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(1350, 1080)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
#endif
	if (!cap.isOpened())
	{
		return;
	}
	for (;;)
	{
		while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER);
		cap >> image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
		src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
#ifdef SAVE_VIDEO
		frame_cnt++;
		if (frame_cnt % 10 == 0)
		{
			frame_cnt = 0;
			// 异步读写加速,避免阻塞生产者
			write_video = std::async(std::launch::async, [&, src]()
									 { writer.write(src); });
		}
#endif
		if (src.empty())
			break;

		image_buffer_front_++;
	}

#endif

#ifdef DAHENG

#ifdef SAVE_VIDEO
	cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(width, height)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
#endif
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (GxCamera::camera_ptr_ != nullptr)
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
			};
			// image_mutex_.lock();
			if (GxCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				// std::cout << "time :" << time_run.count() << std::endl;

				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;

#ifdef SAVE_VIDEO
				frame_cnt++;
				cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
				if (frame_cnt % 10 == 0)
				{
					frame_cnt = 0;
					// 异步读写加速,避免阻塞生产者
					write_video = std::async(std::launch::async, [&, src]()
											 { writer.write(src); });
				}
#endif
			}
			else
			{
				delete GxCamera::camera_ptr_;
				GxCamera::camera_ptr_ = nullptr;
			}
		}
		else
		{
			GxCamera::camera_ptr_ = new DaHengCamera;
			while (!GxCamera::camera_ptr_->StartDevice())
				;
			GxCamera::camera_ptr_->SetResolution();
			while (!GxCamera::camera_ptr_->StreamOn())
				;
			// 设置是否自动白平衡
			GxCamera::camera_ptr_->Set_BALANCE_AUTO(1);
			// 手动设置白平衡通道及系数，此之前需关闭自动白平衡

			GxCamera::camera_ptr_->SetExposureTime(GxCamera::GX_exp_time);
			GxCamera::camera_ptr_->SetGain(3, GxCamera::GX_gain);

			double GX_Gamma = 2.85;
			GxCamera::camera_ptr_->setGamma(GX_Gamma);

			cv::namedWindow("DaHengCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("DaHengExpTime", "DaHengCameraDebug", &GxCamera::GX_exp_time, 10000, GxCamera::DaHengSetExpTime);
			GxCamera::DaHengSetExpTime(0, nullptr);
			cv::createTrackbar("DaHengGain", "DaHengCameraDebug", &GxCamera::GX_gain, 10, GxCamera::DaHengSetGain);
			GxCamera::DaHengSetGain(0, nullptr);
			// GxCamera::DaHengSetGain(0,nullptr);

			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif

#ifdef MIDVISION

#ifdef SAVE_VIDEO
	cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(width, height)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
#endif

	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (MidCamera::camera_ptr_ != nullptr)
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
			};
			// image_mutex_.lock();
			if (MidCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				// std::cout << "time :" << time_run.count() << std::endl;

				MidCamera::camera_ptr_->SetExpose(MidCamera::MV_exp_value);

				bool is = image_mutex_.try_lock();
				if (!is)
				{
					std::cout << "try lock failed!!" << std::endl;
				}
				std::cout << "enter producer lock" << std::endl;
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
				image_mutex_.unlock();
				std::cout << "out producer lock" << std::endl;
#ifdef SAVE_VIDEO
				frame_cnt++;
				cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
				if (frame_cnt % 10 == 0)
				{
					frame_cnt = 0;
					// 异步读写加速,避免阻塞生产者
					write_video = std::async(std::launch::async, [&, src]()
											 { writer.write(src); });
				}
#endif
			}
			else
			{
				delete MidCamera::camera_ptr_;
				MidCamera::camera_ptr_ = nullptr;
			}
		}
		else
		{
			MidCamera::camera_ptr_ = new MVCamera;

			MidCamera::camera_ptr_->SetExpose(5000);

			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("MVExpTime", "MVCameraDebug", &MidCamera::MV_exp_value, 15000, MidCamera::MVSetExpTime);
			// MidCamera::MVSetExpTime(0,nullptr);

			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif
}

std::vector<double> x, y;
void plot_xy()
{
	namespace plt = matplotlibcpp;
	static int count_ = 0;
	count_++;

	if (true)
	{
		x.push_back(count_);
		y.push_back(sin(count_ / 10.0));
		plt::clf();
		plt::plot(x, y);
		plt::pause(0.001);
	}
}

void Factory::consumer()
{
	namespace plt = matplotlibcpp;

#ifdef RECORD_DATA
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1398;
	int height = 1080;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(1398, 1080)); // Avi format，1350，1080
	std::future<void> write_video;
#endif
	plt::ion();
	plt::show();
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		// 若满足这个条件，则让这个函数一只停在这里
		image_mutex_.lock();
		std::cout << "enter consum lock" << std::endl;
		while (image_buffer_front_ <= image_buffer_rear_);
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// 读取最新的图片
		image_buffer_rear_ = image_buffer_front_ - 1;
		// 直接获取引用
		std::cout << "hello" << std::endl;
		cv::Mat &img = image_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
		image_mutex_.unlock();
		std::cout << "out consum lock" << std::endl;
		double src_time = timer_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
		std::cout << "time :" << src_time << std::endl;

#ifdef VIDEO
		BUFF buff;
		if (buffdector.run(img, buff))
		{
		}
#ifdef RECORD_DATA
		frame_cnt++;
		if (true)
		{
			frame_cnt = 0;
			// 异步读写加速,避免阻塞消费者
			write_video = std::async(std::launch::async, [&, img]()
									 { writer.write(img); });
		}
#endif
#else
		// std::vector<ArmorObject> objects;
		std::vector<ArmorObject> objects = infer.run(img);

		if (objects.size() != 0)
		{
			std::pair<Eigen::Vector3d, Eigen::Vector3d> pose = pnp_solver_->poseCalculation(objects[0]);
			coord = pose.first;
			rotation = pose.second;
		}

#endif

		char test[100];
		sprintf(test, "x:%0.4f", coord[0]);
		cv::putText(img, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "y:%0.4f", coord[1]);
		cv::putText(img, test, cv::Point(img.cols / 3, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "z:%0.4f", coord[2]);
		cv::putText(img, test, cv::Point(2 * img.cols / 3, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "roll:%0.4f", rotation[0]*180/CV_PI);
		cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "pitch:%0.4f", rotation[1]*180/CV_PI);
		cv::putText(img, test, cv::Point(img.cols / 3, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "yaw:%0.4f", rotation[2]*180/CV_PI);
		cv::putText(img, test, cv::Point(2 * img.cols / 3, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		std::string windowName = "show";
		cv::namedWindow(windowName, 0);
		cv::imshow(windowName, img);

		cv::waitKey(1);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

		float FPS = 1 / time_run.count();

		std::cout << "                                "
				  << "帧率：" << FPS << std::endl;
	}
}
