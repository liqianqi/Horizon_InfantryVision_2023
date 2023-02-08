#include "include/common.h"
#include "include/thread/thread.h"
#include <thread>
#include <iostream>
int main()
{
    Factory vision;

    std::thread thread1(&Factory::producer,std::ref(vision));
	
    std::thread thread2(&Factory::consumer,std::ref(vision));

    thread1.join();

    thread2.join();

}

// 生产者，消费者模式
