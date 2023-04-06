#ifndef __OV_ENGINE_H__
#define __OV_ENGINE_H__

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

// using namespace ov;

class OpenvinoEngine
{
public:
    OpenvinoEngine();
    ~OpenvinoEngine();

    int Init(const std::string& modelPath);
    int Forward();
    int SetInput(cv::Mat &inframe);
    int GetOutputData(std::vector<float *>& output_blobs);
    int Release();

private:
    std::shared_ptr<ov::Model> network;

    ov::CompiledModel executable_network;
    ov::InferRequest infer_request;
    ov::Tensor input_tensor;
    ov::Tensor output_tensor;

    ov::Shape input_shape;
    ov::Shape output_shape;
    
    ov::element::Type inputPrecision;
    ov::element::Type outputPrecision;

    std::string input_tensor_name;
    std::string output_tensor_name;

    size_t outputNode_size;
};


#endif
