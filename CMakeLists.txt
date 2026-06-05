# CMake 最低版本要求
cmake_minimum_required(VERSION 3.14) #old

project(Horizon_RMVISION_2023)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(OpenCV_DIR /home/robomaster/opencv-4.5.5/build)

set(python_libs "python3.8")
include_directories(/opt/intel/openvino_2022/runtime/include/)
include_directories(${CERES_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Python3 COMPONENTS Interpreter Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} include)

include(FetchContent)
FetchContent_Declare(
   matplotlib_cpp
   GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
   GIT_TAG master
   GIT_SHALLOW TRUE
)
FetchContent_GetProperties(matplotlib_cpp)
if(NOT matplotlib_cpp_POPULATED)
   FetchContent_Populate(matplotlib_cpp)
endif()
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp INTERFACE ${matplotlib_cpp_SOURCE_DIR})
target_link_libraries(matplotlib_cpp INTERFACE Python3::Python Python3::NumPy)

message("finding OpenVINO")
find_package(OpenVINO REQUIRED)
message("finded OpenVINO")
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost COMPONENTS filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
file(GLOB_RECURSE "/usr/lib/python3.8/config-x86_64-linux-gnu/*.so")
#include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/MidVision)
include_directories(${PROJECT_SOURCE_DIR}/serial)

aux_source_directory(./src/thread SRC_FILES)
aux_source_directory(./src/autoaim SRC_FILES)
aux_source_directory(./src/buff SRC_FILES)
aux_source_directory(./serial SRC_FILES)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	set(MV_SDK ${PROJECT_SOURCE_DIR}/MidVision/lib/amd64/libMVSDK.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	set(MV_SDK ${PROJECT_SOURCE_DIR}/MidVision/lib/arm64/libMVSDK.so)
else()
	message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

## 指定生成目标
add_executable(${PROJECT_NAME}  ${SRC_FILES} main.cpp)

target_link_libraries(${PROJECT_NAME} openvino::runtime)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${MV_SDK})
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} matplotlib_cpp)
target_link_libraries(${PROJECT_NAME} -lboost_system ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} -lboost_system pthread)
