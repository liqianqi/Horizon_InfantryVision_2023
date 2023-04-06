#include "ov_engine.h"
#include "base_detector.h"
#include <chrono>

#define MULTI_IMAGE_INFER 0

int main(int argc, char const *argv[])
{
    int ret = 0;
    
    string type;
    string imgpath = "../imgs/0018.png";
    string test_model_path = "../best_FP32/best_model.xml";

    const float Threshold = 0.7;
    std::vector<float *> NetOutput;
    OpenvinoEngine *net = new OpenvinoEngine();
    
    ret = net->Init(test_model_path);
    if(ret != 0) 
	{
        printf("[LOG] init error \n");
        return -1;
    }

    auto start = std::chrono::system_clock::now();

    // detector
    cout << "file name : " << imgpath << endl;
    cv::Mat src_img = cv::imread(imgpath);
    cv::Mat ResizedImg;
    DetectorProcess *PostP = new DetectorProcess();
    ResizedImg = PostP->PreProcessYolo(src_img, 416, 416);
    
    net->SetInput(ResizedImg);

    net->Forward();

    net->GetOutputData(NetOutput);

    PostP->PostProcessYoloV5n(src_img, Threshold, NetOutput);

    auto end = std::chrono::system_clock::now();
    std::cout<<"time use : "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()  <<" ms"<<std::endl;

    delete PostP;
    delete net;

    return 0;
}
