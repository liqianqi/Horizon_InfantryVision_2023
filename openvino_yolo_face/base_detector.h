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
using namespace std;

struct Objects
{
    cv::Rect box;
    float conf;
    float classId;
};

class DetectorProcess 
{
public:
   DetectorProcess();
   ~DetectorProcess();

    cv::Mat PreProcessYolo(cv::Mat &img, int input_h, int input_w);

    int PostProcessYoloV5n(cv::Mat &srcimg, const float threshold, const std::vector<float *> inputData);
private:
    const float netAnchors[3][6] = { {4,5,  8,10,  13,16}, {23,29,  43,55,  73,105},{146,217,  231,300,  335,433} };
    const float netStride[3] = { 8.0, 16.0, 32.0 };
    const int strideSize = 3;   //stride size
    // float nmsThreshold_ = 0.6f;

    // float boxThreshold = 0.6;
	// float classThreshold = 0.6;

	float nmsThreshold = 0.3;
	float nmsScoreThreshold = 0.5;

    int classNum = 36;   // set class num
    std::vector<cv::Rect2f> origin_rect;
    std::vector<float>  origin_confidence;
    std::vector<float>  origin_cls_id;
};


#endif
