#include "../../include/autoaim/ov_engine.h"

OpenvinoEngine::OpenvinoEngine()
{
    ov::Core core;
    std::vector<std::string> availableDevices = core.get_available_devices();
    if(availableDevices.empty()) {
        std::cout << "Unsupport device" << std::endl;
    }

    inputPrecision = ov::element::f32;   // set input/output precision
    outputPrecision = ov::element::f32;

    network = core.read_model("/home/liqianqi/Horizon_InfantryVision-2023/best_model.xml");

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

OpenvinoEngine::~OpenvinoEngine()
{
    Release();
}

int OpenvinoEngine::Forward()
{
    infer_request.infer();
    return 0;
}

int OpenvinoEngine::SetInput(cv::Mat &inframe)
{
    uint8_t *input_data = (uint8_t *)inframe.data;
    input_tensor = ov::Tensor(executable_network.input().get_element_type(), executable_network.input().get_shape(), input_data);
    infer_request.set_input_tensor(input_tensor);
    return 0;
}


int OpenvinoEngine::GetOutputData(std::vector<float *>& output_blobs)
{
    // ov::Tensor output_tensor;
    for (int idx = 0; idx < outputNode_size; idx++)
    {
        output_tensor = infer_request.get_output_tensor(idx);
        float *detections = output_tensor.data<float>();
        output_blobs.push_back(detections);
    }
    return 0;
}

int OpenvinoEngine::Release()
{
    return 0;
}


