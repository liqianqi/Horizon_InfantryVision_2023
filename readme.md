# 四点模型
## 训练
- 直接 yolov5-face 按照上交格式训练即可,考虑减少类别，要是太麻烦就不换了

## pt转onnx
- 利用 yolov5-face 的 export.py转换onnx，注意: 转换过的onnx已经包含后处理操作，无需再次在推理进行anchor、stride、sigmod 等操作。（只要用netron导出的onnx是三分支合一的操作就是包含后处理操作）

## onnx 转 IR
- python3 -m venv openvino_env
- source openvino_env/bin/activate
- mo --input_model ./best_model.onnx --data_type FP32 --input_shape [1,3,416,416] --output_dir ./output_model --scale 255     
- (--scale 255 是必要的)

## 直接推理
- 推理在include和src中的autoaim中,Inference.h/Inference.cpp已经弃用。

# 更新记录
- V0.1 Beta 完成自瞄推理部分. 2023.2.9
- V1.1 Beta 新加预测部分，2023.2.16
- V1.2 Beta 增加midvision 2023.2.17

# 待测试
- PNP姿态，相对于相机和云台的姿态
- 逻辑部分的BUG
- 反陀螺编写
- 迈德威视相机模块编入(这几天写出来)
- 大能量机关编写
