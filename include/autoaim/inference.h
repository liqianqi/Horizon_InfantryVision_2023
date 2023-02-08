#pragma once

#include "../common.h"
#include <string>
#include <thread>
#include <future>
// static constexpr int INPUT_W = 640;    // Width of input
// static constexpr int INPUT_H = 384;    // Height of input
static constexpr int INPUT_W = 416;       // Width of input
static constexpr int INPUT_H = 416;       // Height of input
static constexpr int NUM_CLASSES = 36;    // Number of classes
static constexpr int NUM_COLORS = 8;      // Number of classes
static constexpr int TOPK = 128;          // TopK
static constexpr float NMS_THRESH = 0.3;
static constexpr float BBOX_CONF_THRESH = 0.75;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.9;

using namespace ov::preprocess;

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

struct ArmorObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};

class Inference
{
public:

    Inference()
    {
        std::string modelPath = "/home/liqianqi/Horizon_InfantryVision-2023/best_model.xml";
        std::vector<std::string> availableDevices = core.get_available_devices();
        if(availableDevices.empty()) {
            std::cout << "Unsupport device" << std::endl;
        }
        // for (int i = 0; i < availableDevices.size(); i++) {
        //     printf("supported device name : %s \n", availableDevices[i].c_str());
        // }

        inputPrecision = ov::element::f32;   // set input/output precision
        outputPrecision = ov::element::f32;

        network = core.read_model(modelPath);

        ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(network);

        input_tensor_name = network->input().get_any_name();

        ppp.input(input_tensor_name).tensor()     // for input Tensor
                                    .set_element_type(ov::element::u8)
                                    .set_layout("NHWC")
                                    .set_color_format(ov::preprocess::ColorFormat::BGR);

        ppp.input(input_tensor_name).preprocess()
                                    .convert_element_type(inputPrecision);
                                    // .convert_color(ov::preprocess::ColorFormat::BGR);

        ppp.input(input_tensor_name).model()
                                    .set_layout("NCHW");

        for (auto&& output : network->outputs())
        {
            output_tensor_name = output.get_any_name();
            ppp.output(output_tensor_name).tensor()
                                          .set_element_type(outputPrecision);
        }
        network = ppp.build();

        input_shape = network->input().get_shape();

        outputNode_size = network->outputs().size();
        executable_network = core.compile_model(network, "CPU"); // SET DEVICE
        infer_request = executable_network.create_infer_request();

    }
    ~Inference(){}

public:

    ov::Core core;
    std::string input_name;
    std::string output_name;

    std::shared_ptr<ov::Model> network;// 网络
    ov::CompiledModel executable_network;

    ov::InferRequest infer_request;// 推理请求
    ov::Tensor input_tensor;
    ov::Tensor output_tensor;

    ov::Shape input_shape;
    ov::Shape output_shape;

    ov::element::Type inputPrecision;
    ov::element::Type outputPrecision;

    std::string input_tensor_name;
    std::string output_tensor_name;

    size_t outputNode_size;
    const float anchors[3][6] = { {4,5,  8,10,  13,16},
                                  {23,29,  43,55,  73,105},
                                  {146,217,  231,300,  335,433} };
public:

    static inline int argmax(const float *ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] > ptr[max_arg]) max_arg = i;
        }
        return max_arg;
    }

    static inline float intersection_area(const ArmorObject& a, const ArmorObject& b)
    {
        cv::Rect_<float> inter = a.rect & b.rect;
        return inter.area();
    }

    float calcTriangleArea(cv::Point2f pts[3])
    {
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;

        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    float calcTetragonArea(cv::Point2f pts[4])
    {
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }
    static inline float sigmoid_x(float x)
    {
        return static_cast<float>(1.f / (1.f + std::exp(-x)));
    }

    cv::Mat scaledResize(cv::Mat& img , Eigen::Matrix<float,3,3>& transform_matrix);
    bool detect(cv::Mat &src,std::vector<ArmorObject>& objects);
    void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides);
    void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr,
                                        Eigen::Matrix<float,3,3> &transform_matrix,float prob_threshold,
                                        std::vector<ArmorObject>& objects);
    void qsort_descent_inplace(std::vector<ArmorObject>& faceobjects, int left, int right);
    void qsort_descent_inplace(std::vector<ArmorObject>& objects);
    void nms_sorted_bboxes(std::vector<ArmorObject>& faceobjects, std::vector<int>& picked,
                                float nms_threshold);
    void decodeOutputs(std::vector<float *>& prob, std::vector<ArmorObject>& objects,
                                Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h);


    void run(cv::Mat &img);

};
