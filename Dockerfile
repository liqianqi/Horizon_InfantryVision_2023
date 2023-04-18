FROM ericfengx/rm_robot

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

# 设置环境变量
ENV LD_LIBRARY_PATH=/opt/intel/openvino_2022/deployment_tools/inference_engine/lib/intel64:/usr/local/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
ENV PYTHONPATH=/usr/local/lib/python3.8/dist-packages:$PYTHONPATH

# 运行程序
CMD ["./build/app"]
