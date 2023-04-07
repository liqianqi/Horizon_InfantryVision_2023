# 基于的基础镜像
FROM openvino/ubuntu20_dev:2022.3

# 维护者信息
MAINTAINER cai 2041671738@qq.com

# 代码添加到 app 文件夹
# ADD $PWD /app

COPY ./requirements.txt ./
# 设置 /app 文件夹是工作目录
# WORKDIR /app

#管理员权限
USER root

# 安装相关支持
RUN pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

# 声明镜像内服务监听的端口
EXPOSE 5050

# docker启动后运行的命令
CMD /bin/bash -c "source /opt/intel/openvino/bin/setupvars.sh"
ENTRYPOINT ["gunicorn", "-c", "gunicorn_conf.py", "run_openvino:app"]
