# 四点模型
## 训练
- 直接yolov5-face按照上交格式转即可

## pt转onnx
- 利用yolov5-face的export.py转换onnx，注意: 转换过的onnx已经包含后处理操作，无需再次在推理进行anchor、stride、sigmod等操作。（只要用netron导出的onnx是三分支合一的操作就是包含后处理操作）

## onnx 转 IR
mo --input_model ./best_model.onnx --data_type FP32 --input_shape [1,3,416,416] --output_dir ./output_model --scale 255 (--scale 255 是必要的)

## 直接推理
