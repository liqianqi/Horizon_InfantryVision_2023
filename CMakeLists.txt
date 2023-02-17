# CMake 最低版本要求
cmake_minimum_required(VERSION 3.14) #old

project(Horizon_RMVISION_2023)

if(EXISTS /usr/bin/clang AND EXISTS /usr/bin/clang++)
   set(CMAKE_C_COMPILER /usr/bin/clang)
   set(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif()

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE debug)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# C++17 标准、o3优化、多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -pthread")


# 库文件的放置目录
set(OpenCV_DIR /home/liqianqi/opencv-4.5.5/build)
set(SDK_DIR /home/liqianqi/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/libgxiapi.so)

set(python_libs "python3.8")
include_directories(/opt/intel/openvino_2022/runtime/include/)
include_directories(${Python3_INCLUDE_DIRS} include)
include_directories(${CERES_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include "/usr/include/python3.8")

find_package(Python3 COMPONENTS Intrepreter Development NumPy)
message("finding OpenVINO")
find_package(OpenVINO REQUIRED)
message("finded OpenVINO")
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(matplotlib_cpp REQUIRED)
# Ceres
find_package(Ceres REQUIRED)
file(GLOB_RECURSE "/usr/lib/python3.8/config-x86_64-linux-gnu/*.so")
#include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/DaHeng)
include_directories(${PROJECT_SOURCE_DIR}/MidVision)

aux_source_directory(./src/thread SRC_FILES)
aux_source_directory(./src/autoaim SRC_FILES)
aux_source_directory(./src/buff SRC_FILES)
aux_source_directory(./DaHeng SRC_FILES)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	set(MV_SDK /home/liqianqi/Horizon_InfantryVision-2023/MidVision/lib/amd64/libMVSDK.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	set(MV_SDK /home/liqianqi/Horizon_InfantryVision-2023/MidVision/lib/arm64/libMVSDK.so)
else()
	message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

## 指定生成目标
add_executable(${PROJECT_NAME}  ${SRC_FILES} main.cpp)

target_link_libraries(${PROJECT_NAME} openvino::runtime)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${SDK_DIR})
target_link_libraries(${PROJECT_NAME} ${MV_SDK})
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} matplotlib_cpp::matplotlib_cpp)
target_link_libraries(${PROJECT_NAME} libpython3.8.so)
