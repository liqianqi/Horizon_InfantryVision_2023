cmake_minimum_required(VERSION 3.5)
project(openvinoSdk)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -pthread -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

# opencv 
find_package(OpenCV REQUIRED)

# OpenVINO
set(OpenVINO_DIR /opt/intel/openvino_2022/runtime/cmake/)
find_package(OpenVINO REQUIRED)


add_library(common SHARED ${CMAKE_CURRENT_SOURCE_DIR}/base_detector.cpp 
                           ${CMAKE_CURRENT_SOURCE_DIR}/ov_engine.cpp)
                           
target_include_directories(common PUBLIC ${OpenCV_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

target_link_libraries(common ${OpenCV_LIBS} openvino::runtime)

add_executable(test main.cpp)
target_link_libraries(test common)