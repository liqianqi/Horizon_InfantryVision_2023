FROM ericfengx/rm_robot

# 安装依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    libeigen3-dev \
    libceres-dev \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-matplotlib \
    wget

# 安装OpenVINO Toolkit
RUN wget https://registrationcenter-download.intel.com/akdlm/irc_nas/19325/l_openvino_toolkit_p_2022.3.0.242.tgz && \
    tar -xvf l_openvino_toolkit_p_2022.3.0.242.tgz && \
    cd l_openvino_toolkit_p_2022.3.0.242 && \
    ./install.sh && \
    cd .. && \
    rm -rf l_openvino_toolkit_p_2022.3.0.242*

# 安装大恒迈德威视相机驱动
RUN git clone https://github.com/MVS-DRIVER/mvSDK.git && \
    cd mvSDK && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf mvSDK

# 安装OpenCV
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf opencv

# 安装matplotlib-cpp
RUN git clone https://github.com/lava/matplotlib-cpp.git && \
    cd matplotlib-cpp && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf matplotlib-cpp

# 指定工作目录
WORKDIR /app

# 将当前目录下的文件复制到工作目录中
COPY . .

# 构建程序
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make

# 设置环境变量
ENV LD_LIBRARY_PATH=/opt/intel/openvino_2022.3.0.242/deployment_tools/inference_engine/lib/intel64:/usr/local/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
ENV PYTHONPATH=/usr/local/lib/python3.8/dist-packages:$PYTHONPATH

# 运行程序
CMD ["./build/app"]
