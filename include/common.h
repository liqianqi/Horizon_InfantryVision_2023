#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include "openvino/openvino.hpp"
#include "openvino/opsets/opset9.hpp"
#include <matplotlibcpp.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <termio.h>
#include<algorithm>

namespace lqq
{
    template <int MAX_SIZE, class T>
    class deque
    {
    public:
        deque() : _front(0), _rear(0), buffer_space(nullptr)
        {
            buffer_space = (T *)malloc(MAX_SIZE * sizeof(T));
        }
        ~deque()
        {
            free(buffer_space);
        }

    private:
        int size;
        int _front;
        int _rear;
        T *buffer_space;

    public:
        bool Insert(const T &x);
        bool Delete();
    };

    template <int MAX_SIZE, class T>
    bool deque<MAX_SIZE, T>::Insert(const T &x)
    {
        if ((_rear + 1) % MAX_SIZE == _front) // 队列满
        {
            std::cout << "队列已满！" << std::endl;
            return false;
        }

        buffer_space[_rear] = x;

        _rear = (_rear + 1) % MAX_SIZE;
        return true;
    }

    template <int MAX_SIZE, class T>
    bool deque<MAX_SIZE, T>::Delete()
    {
        if (_front == _rear)
        {
            std::cout << "队列为空！" << std::endl;
            return false;
        }
        _front = (_front + 1) % MAX_SIZE;
        return true;
    }

}
static long now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
class GimbalPose
{
public:
    float pitch;
    float yaw;
    float roll;
    double timestamp;
    // 初始化函数
    GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
    {
        this->pitch = pitch;
        this->yaw = yaw;
        this->roll = roll;
    }
    // 左值
    GimbalPose operator=(const GimbalPose &gm)
    {
        this->pitch = gm.pitch;
        this->yaw = gm.yaw;
        this->roll = gm.roll;
        this->timestamp = gm.timestamp;
        return *this;
    }

    GimbalPose operator=(const float init_value)
    {
        this->pitch = init_value;
        this->yaw = init_value;
        this->roll = init_value;
        this->timestamp = now();
        return *this;
    }

    friend GimbalPose operator-(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch - gm2.pitch;
        temp.yaw = gm1.yaw - gm2.yaw;
        temp.roll = gm1.roll - gm2.roll;
        temp.timestamp = now();
        return temp;
    }

    friend GimbalPose operator+(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch + gm2.pitch;
        temp.yaw = gm1.yaw + gm2.yaw;
        temp.roll = gm1.roll + gm2.roll;
        temp.timestamp = now();
        return temp;
    }

    friend GimbalPose operator*(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        temp.timestamp = now();
        return temp;
    }

    friend GimbalPose operator*(const float k, const GimbalPose &gm)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        temp.timestamp = now();
        return temp;
    }

    friend GimbalPose operator/(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch / k;
        temp.yaw = gm.yaw / k;
        temp.roll = gm.roll / k;
        temp.timestamp = now();
        return temp;
    }

    friend std::ostream &operator<<(std::ostream &out, const GimbalPose &gm)
    {
        out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]" << std::endl;
        return out;
    }
};
