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
#include "common.h"
using namespace std;

static constexpr int INPUT_W = 416; // Width of input
static constexpr int INPUT_H = 416; // Height of input

enum class COLOR
{
    RED,
    BLUE,
    GRAY
};

struct ArmorObject
{
    cv::Rect rect;
    int confidence;
    float label;
    COLOR color;
    cv::Point2f pts[4];
    Eigen::Vector3d coord;
    Eigen::Vector3d pyr;
};

class DetectorProcess
{
public:
    DetectorProcess();
    ~DetectorProcess();

    cv::Mat PreProcessYolo(cv::Mat &img, int input_h, int input_w);

    int PostProcessYoloV5n(cv::Mat &srcimg, const float threshold, const std::vector<float *> inputData, std::vector<ArmorObject> &objects);

    std::vector<ArmorObject> run(cv::Mat &img);

    COLOR color_judge(cv::Rect &rect, cv::Mat &img)
    {
        using namespace cv;
        COLOR color_id_;
        // Avoid assertion failed
        if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= img.cols && 0 <= rect.y &&
            0 <= rect.height && rect.y + rect.height <= img.rows)
        {
            int sum_r = 0, sum_b = 0;
            auto roi = img(rect);
            // Iterate through the ROI
            for (int i = 0; i < roi.rows; i++)
            {
                for (int j = 0; j < roi.cols; j++)
                {
                    // if point is inside contour
                    sum_r += roi.at<cv::Vec3b>(i, j)[2];
                    sum_b += roi.at<cv::Vec3b>(i, j)[0];
                }
            }
            // Sum of red pixels > sum of blue pixels ?
            std::cout << "r : " << sum_r << " b " << sum_b << std::endl;
            color_id_ = sum_r > sum_b ? COLOR::RED : COLOR::BLUE;
            return color_id_;
        }
        else
        {
            return COLOR::GRAY;
        }
    }

private:
    const float netAnchors[3][6] = {{4, 5, 8, 10, 13, 16},
                                    {23, 29, 43, 55, 73, 105},
                                    {146, 217, 231, 300, 335, 433}};
    const float netStride[3] = {8.0, 16.0, 32.0};
    const int strideSize = 3; // stride size
    // float nmsThreshold_ = 0.6f;

    // float boxThreshold = 0.6;
    // float classThreshold = 0.6;

    float nmsThreshold = 0.3;
    float nmsScoreThreshold = 0.5;
    std::shared_ptr<OpenvinoEngine> net = std::make_shared<OpenvinoEngine>();
    // std::vector<ArmorObject> objects;
    int classNum = 36; // set class num
    std::vector<cv::Rect2f> origin_rect;
    std::vector<float> origin_confidence;
    std::vector<float> origin_cls_id;

public:
    COLOR color_id_;
    COLOR last_color_id_;
    COLOR set_color_;
};

#endif
