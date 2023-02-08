#include "../../include/autoaim/base_detector.h"

DetectorProcess::DetectorProcess()
{

}

DetectorProcess::~DetectorProcess()
{

}

static bool sort_score(std::vector<float> box1,std::vector<float> box2)
{
	return box1[4] > box2[4] ? true : false;
}

static inline float sigmoid_x(float x)
{
	return static_cast<float>(1.f / (1.f + exp(-x)));
}
static inline int argmax(const float *ptr, int len)
{
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}

cv::Mat DetectorProcess::PreProcessYolo(cv::Mat &img, int input_h, int input_w)
{   
    cv::Mat RGB_img;
    cvtColor(img, RGB_img, cv::COLOR_BGR2RGB);
    cv::Mat re(input_h, input_w, CV_8UC3);
    cv::resize(RGB_img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    return re;
}
void drawPred(float conf, int left, int top, int right, int bottom, cv::Mat& frame, vector<int> landmark)   // Draw the predicted bounding box
{
	//Draw a rectangle displaying the bounding box
	cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);

	//Get the label for the class name and its confidence
	string label = cv::format("%.2f", conf);

	//Display the label at the top of the bounding box
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);

	cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 1);
	for (int i = 0; i < 5; i++)
	{
        printf("[%d,%d]\n",landmark[2 * i], landmark[2 * i + 1]);
        //cv::circle(frame, cv::Point(landmark[2 * i], landmark[2 * i + 1]), 5, cv::Scalar(0, 0, 255), -1);
        cv::line(frame, cv::Point(landmark[0], landmark[1]), cv::Point(landmark[2], landmark[3]),cv::Scalar(193, 182, 255), 1, 8);
        cv::line(frame, cv::Point(landmark[2], landmark[3]), cv::Point(landmark[4], landmark[5]),cv::Scalar(193, 182, 255), 1, 8);
        cv::line(frame, cv::Point(landmark[4], landmark[5]), cv::Point(landmark[6], landmark[7]),cv::Scalar(193, 182, 255), 1, 8);
        cv::line(frame, cv::Point(landmark[6], landmark[7]), cv::Point(landmark[0], landmark[1]),cv::Scalar(193, 182, 255), 1, 8);
	}
    //cv::imwrite("detect.jpg",frame);
}

int DetectorProcess::PostProcessYoloV5n(cv::Mat &srcimg, const float threshold, 
                                    const std::vector<float *> inputData)
{
    int NetInputHeight=416, NetInputWidth=416;
    float* Idata = static_cast<float*>(inputData[0]);
	vector<float> confidences;
	vector<cv::Rect> boxes;
    vector< vector<int>> landmarks;

	float ratioh = (float)srcimg.rows / NetInputHeight;
    float ratiow = (float)srcimg.cols / NetInputWidth;

    int n = 0, q = 0, i = 0, j = 0, nout = 51, row_ind = 0, k = 0; ///xmin,ymin,xamx,ymax,box_score,x1,y1, ... ,x5,y5,face_score
	for (n = 0; n < 3; n++)   ///特征图尺度
	{
		int num_grid_x = (int)(NetInputWidth / netStride[n]);
		int num_grid_y = (int)(NetInputHeight / netStride[n]);
        for(q = 0; q < 3; q++)    ///anchor
		{
			for (i = 0; i < num_grid_y; i++)
			{
				for (j = 0; j < num_grid_x; j++)
				{
					float* pdata = Idata + row_ind * nout;
                    float box_score = (pdata[4]);
                    if (box_score > 0.6)
					{
                        int id = argmax(pdata + 15, 36);
                        float face_score = pdata[15 + id];
                        if(face_score < 0.5)
                            continue;

                        float cx = pdata[0]*ratiow;  ///cx
                        float cy = pdata[1]*ratioh ;   ///cy
                        float w = pdata[2];   ///w
                        float h = pdata[3];  ///h

                        int left = cx - 0.5*w;
                        int top = cy - 0.5*h;

						confidences.push_back(face_score);

						vector<int> landmark(10);
                        for (k = 5; k < 13; k+=2)
						{
							const int ind = k - 5;
                            landmark[ind] = (int)(pdata[k])*ratiow;
                            landmark[ind + 1] = (int)(pdata[k + 1])*ratioh;
						}
                        for (int i = 0; i < 4; i++)
                        {
                            cv::Point a = cv::Point(landmark[2 * i], landmark[2 * i + 1]);
                            //circle(frame, a, 1, Scalar(0, 255, 0), -1);
                        }
                        cv::Point apex[4];
                        for (int i = 0; i < 4; i++)
                        {
                            apex[i] = cv::Point(landmark[2 * i], landmark[2 * i + 1]);
                        }

                        std::vector<cv::Point2f> tmp(apex, apex + 4);
                        boxes.push_back(cv::boundingRect(tmp));
						landmarks.push_back(landmark);
						
					}
					row_ind++;
				}
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	cv::dnn::NMSBoxes(boxes, confidences, nmsScoreThreshold, nmsThreshold, indices);
    printf("indices size = %ld\n", indices.size());
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
        printf("idx = %d\n",idx);
        cv::Rect bbox = boxes[idx];
        
		drawPred(confidences[idx], bbox.x, bbox.y,
			        bbox.x + bbox.width, bbox.y + bbox.height, srcimg, landmarks[idx]);
	}
    return 0;
}


void DetectorProcess::run(cv::Mat &img)
{
    const float Threshold = 0.7;
    std::vector<float *> NetOutput;
    cv::Mat ResizedImg;

    ResizedImg = PreProcessYolo(img, 416, 416);

    net->SetInput(ResizedImg);

    net->Forward();

    net->GetOutputData(NetOutput);

    PostProcessYoloV5n(img, Threshold, NetOutput);

}
