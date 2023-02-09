#ifndef __BASE_DETECTOR_H__
#define __BASE_DETECTOR_H__

#include <float.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <utility>
#include "ov_engine.h"
using namespace std;

class ArmorObject
{
public:
    cv::Rect rect;
    int confidence;
    float label;
    cv::Point pts[4];
};

class DetectorProcess 
{
public:
   DetectorProcess();
   ~DetectorProcess();

    cv::Mat PreProcessYolo(cv::Mat &img, int input_h, int input_w);

    int PostProcessYoloV5n(cv::Mat &srcimg, const float threshold, const std::vector<float *> inputData,std::vector<ArmorObject>& objects);

    std::vector<ArmorObject> run(cv::Mat &img);
private:
    const float netAnchors[3][6] = { {4,5,  8,10,  13,16}, {23,29,  43,55,  73,105},{146,217,  231,300,  335,433} };
    const float netStride[3] = { 8.0, 16.0, 32.0 };
    const int strideSize = 3;   //stride size
    // float nmsThreshold_ = 0.6f;

    // float boxThreshold = 0.6;
	// float classThreshold = 0.6;

	float nmsThreshold = 0.3;
	float nmsScoreThreshold = 0.5;
    std::shared_ptr<OpenvinoEngine> net = std::make_shared<OpenvinoEngine>();
    //std::vector<ArmorObject> objects;
    int classNum = 36;   // set class num
    std::vector<cv::Rect2f> origin_rect;
    std::vector<float>  origin_confidence;
    std::vector<float>  origin_cls_id;
};


#endif
