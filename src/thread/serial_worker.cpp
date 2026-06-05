#include "../../include/thread/thread.h"

bool Factory::openSerial()
{
	fd = OpenPort("/dev/ttyUSB0");
	if (fd == -1)
	{
		fd = OpenPort("/dev/ttyUSB1");
	}

	if (fd == -1)
	{
		return false;
	}

	configureSerial(fd);
	return true;
}

void Factory::receiveSerialFrame()
{
	if (fd == -1)
	{
		std::this_thread::yield();
		return;
	}

	std::lock_guard<std::mutex> lock(serial_mutex_);
	data_controler_.getData(fd, stm32data_temp);
	is_aim_ = stm32data_temp.is_aim;

	if (!stm32data_temp.dubug_print)
	{
		stm32data = last_stm32_;
		return;
	}

	last_stm32_ = stm32data_temp;
	if (MCU_data_.size() < mcu_size_)
	{
		MCU_data_.push_back(stm32data_temp);
	}
	else
	{
		MCU_data_.pop_front();
		MCU_data_.push_back(stm32data_temp);
	}
}
