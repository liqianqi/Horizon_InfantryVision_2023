#ifndef SEND_RECEIVE_H
#define SEND_RECEIVE_H

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <errno.h>
#include <sys/ioctl.h>
#include "../include/common.h"
#include <cmath>
#include <math.h>
#include <stdio.h>
namespace Horizon{
using namespace std;
//传输数据用的类型三
typedef union
{
    float f;
    unsigned char c[4];
}float2uchar;

typedef union{
    int f;
    unsigned char c[4];
}int2uchar;

#define aimMOD 0x01
#define topMOD 0x02
#define	smallWindmill 0x04
#define bigWindmill 0x06

//这里就是要把数字x的第n位（bit（n-1）位）置为1
//1U就表示的是无符号的1，宏定义可以传参的
#define SET_BIT(x,n)    (x=(x | (1U<<(n-1))))

//这里就是要把数字x的第n位（bit（n-1）位）清零
#define CLEAR_BIT(x,n)    (x=(x & (~(1U<<(n-1)))))

class DataControler{
public:
    DataControler():state_(0){}
    // 视觉向电控传数据
    #pragma pack (1)
    struct VisionData
	{
        float2uchar pitch_data_;
        float2uchar yaw_data_;
        float2uchar pitch_speed_data_;
        float2uchar yaw_speed_data_;
        int2uchar time;
        unsigned char OnePointFive;
        bool is_gyro_;
        bool is_fire;
        unsigned int firing_frequency_;
        bool is_have_armor;
        float2uchar pitch_w_rads;
        float2uchar yaw_w_rads;
        VisionData(){
            pitch_data_.f = 0;
            yaw_data_.f = 0;
            is_fire = false;
            is_gyro_ = false;
            firing_frequency_ = 0;
            pitch_speed_data_.f = 0;
            yaw_speed_data_.f = 0;
            time.f = 0;
            OnePointFive = 0;
            is_have_armor = 0;
            pitch_w_rads.f = -1;
            yaw_w_rads.f = -1;
            //aim_energy_bit_.f = 0;
        }
    };
    #pragma pack ()

    // 电控向视觉发送数据
    #pragma pack (1)
    struct Stm32Data{
        float2uchar pitch_data_;
        float2uchar yaw_data_;
        float init_firing_rate;
        float2uchar pitch_speed_data_;
        float2uchar yaw_speed_data_;
        int2uchar time;
        unsigned char OnePointFive;// 电控视觉状态标志位
        int p_bit_;
        int y_bit_;
        bool IsHave;
        int flag; // 自瞄和能量机关切换标志位
        uint8_t aim_bit_;
        bool dubug_print;
        bool is_aim;
        bool color_;// true是蓝色，false是红色
        Stm32Data(){
            OnePointFive = {0};
            pitch_data_.f = 0;
            yaw_data_.f = 0;
            init_firing_rate = 0;
            pitch_speed_data_.f = 0;
            yaw_speed_data_.f = 0;
            time.f = 0;
            //aim_energy_bit_.f = 0;
            IsHave = false;
            flag = 0;
			dubug_print = false;
            //dubug_print = true;
        }
        // 以字节传输数据
    };
    #pragma pack ()

public:
    int getBit(unsigned char b,int i)
    {
        int bit = (int) ((b >> (i-1)) & 0x1);
        return bit;
    }
    void sentData(int fd,VisionData data);
    void getData(int fd,Stm32Data &get_data);
public:
    int state_;
    bool is_start_time_stamp_;      // is stamp time

};

int OpenPort(const char *Portname);
int configureSerial(int fd);

}

#endif // SEND_RECEIVE_H