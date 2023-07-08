#include "../../include/buff/base_detector_energy.h"

// Eigen::Matrix<float, 3, 3> transform_matrix;
namespace BUFF
{
float ratio_ = 0;
int x_offset = 0;
int y_offset = 0;
DetectorProcess::DetectorProcess()
{
}

DetectorProcess::~DetectorProcess()
{
}

static bool sort_score(std::vector<float> box1, std::vector<float> box2)
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
	for (int i = 1; i < len; i++)
	{
		if (ptr[i] > ptr[max_arg])
			max_arg = i;
	}
	return max_arg;
}

cv::Mat DetectorProcess::PreProcessYolo(cv::Mat &img, int input_h, int input_w)
{
	cv::Mat RGB_img;
	cvtColor(img, RGB_img, cv::COLOR_BGR2RGB);
	float r = std::min(INPUT_W_ENERGY / (img.cols * 1.0), INPUT_H_ENERGY / (img.rows * 1.0));
	ratio_ = r;
	int unpad_w = r * img.cols;
	int unpad_h = r * img.rows;

	int dw = INPUT_W_ENERGY - unpad_w;
	int dh = INPUT_H_ENERGY - unpad_h;

	dw /= 2;
	dh /= 2;

	x_offset = dw;
	y_offset = dh;

	// transform_matrix << 1.0 / r, 0, -dw / r,
	//                     0, 1.0 / r, -dh / r,
	//                     0, 0, 1;

	cv::Mat re;
	cv::resize(RGB_img, re, cv::Size(unpad_w, unpad_h));
	cv::Mat out;
	cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
	// std::cout << "width: " << out.cols << "	" << "height: " << out.rows << std::endl;
	// cv::imshow("1",out);
	return out;
}

void drawPred(float conf, int left, int top, int right, int bottom, cv::Mat &frame, vector<int> landmark, int id) // Draw the predicted bounding box
{
	// Draw a rectangle displaying the bounding box
	// cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
	// cv::rectangle(frame, cv::Point(left-70, top-70), cv::Point(right+30, bottom+30), cv::Scalar(0, 255, 0), 2);

	// Get the label for the class name and its confidence
	string label = cv::format("%.2f", conf);

	// Display the label at the top of the bounding box
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 1);

	string id_armor = cv::format("%.2d", id);
	cv::Size idSize = cv::getTextSize(id_armor, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, idSize.height);
	cv::putText(frame, id_armor, cv::Point(left + 80, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 1);

	for (int i = 0; i < 5; i++)
	{
		printf("[%d,%d]\n", landmark[2 * i], landmark[2 * i + 1]);
		if(true)
		{
			cv::circle(frame, cv::Point(landmark[2 * i], landmark[2 * i + 1]), 10, cv::Scalar(0, 255, 0), -1);
		}
	}
	cv::line(frame, cv::Point(landmark[0], landmark[1]), cv::Point(landmark[2], landmark[3]), cv::Scalar(211,160,211), 3, 8);
	cv::line(frame, cv::Point(landmark[2], landmark[3]), cv::Point(landmark[4], landmark[5]), cv::Scalar(211,160,211), 3, 8);
	cv::line(frame, cv::Point(landmark[4], landmark[5]), cv::Point(landmark[6], landmark[7]), cv::Scalar(211,160,211), 3, 8);
	cv::line(frame, cv::Point(landmark[6], landmark[7]), cv::Point(landmark[8], landmark[9]), cv::Scalar(211,160,211), 3, 8);
	cv::line(frame, cv::Point(landmark[8], landmark[9]), cv::Point(landmark[0], landmark[1]), cv::Scalar(211,160,211), 3, 8);

	// cv::imwrite("detect.jpg",frame);
}

int DetectorProcess::PostProcessYoloV5n(cv::Mat &srcimg, const float threshold,
										const std::vector<float *> inputData,
										std::vector<BuffObject> &objects)
{
	int NetInputHeight = 416, NetInputWidth = 416;
	float *Idata = static_cast<float *>(inputData[0]);
	vector<float> confidences;
	vector<cv::Rect> boxes;
	vector<vector<int>> landmarks;
	vector<int> labels;

	float ratioh = (float)srcimg.rows / NetInputHeight;
	float ratiow = (float)srcimg.cols / NetInputWidth;

	int n = 0, q = 0, i = 0, j = 0, nout = 21, row_ind = 0, k = 0; /// xmin,ymin,xamx,ymax,box_score,x1,y1, ... ,x5,y5,face_score
	for (n = 0; n < 3; n++)										   /// 特征图尺度
	{
		int num_grid_x = (int)(NetInputWidth / netStride[n]);
		int num_grid_y = (int)(NetInputHeight / netStride[n]);
		for (q = 0; q < 3; q++) /// anchor
		{
			for (i = 0; i < num_grid_y; i++)
			{
				for (j = 0; j < num_grid_x; j++)
				{
					float *pdata = Idata + row_ind * nout;
					row_ind++;
					float box_score = (pdata[4]);
					if (box_score > 0.70)
					{
						int id = argmax(pdata + 15, 6);
						float face_score = pdata[15 + id];
						if (face_score < 0.65)
							continue;

						if(id == 2 || id == 0 || id == 5 || id == 3)
						{
							continue;
						}

						float cx = pdata[0] * ratiow; /// cx
						float cy = pdata[1] * ratioh; /// cy
						float w = pdata[2];			  /// w
						float h = pdata[3];			  /// h

						int left = cx - 0.5 * w;
						int top = cy - 0.5 * h;

						confidences.push_back(face_score);

						vector<int> landmark(10);
						for (k = 5; k < 15; k += 2)
						{
							const int ind = k - 5;
							landmark[ind] = (int)(pdata[k] - x_offset) / ratio_;
							landmark[ind + 1] = (int)(pdata[k + 1] - y_offset) / ratio_;
						}

						cv::Point apex[5];
						for (int i = 0; i < 5; i++)
						{
							apex[i] = cv::Point(landmark[2 * i], landmark[2 * i + 1]);
						}

						std::vector<cv::Point2f> tmp(apex, apex + 5);
						boxes.push_back(cv::boundingRect(tmp));
						landmarks.push_back(landmark);
						labels.emplace_back(id);
					}
				}
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	cv::dnn::NMSBoxes(boxes, confidences, nmsScoreThreshold, nmsThreshold, indices);
	printf("indices size = %ld\n", indices.size());

	BuffObject obj;
	int count = indices.size();
	objects.resize(count);
	objects.clear();
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		if(labels[idx] != 5)
		{
			//continue;
		}
		printf("idx = %d\n", labels[idx]);
		cv::Rect bbox = boxes[idx];
		drawPred(confidences[idx], bbox.x, bbox.y,
				 bbox.x + bbox.width, bbox.y + bbox.height, srcimg, landmarks[idx], labels[idx]);
		obj.rect = bbox;
		obj.label = labels[idx];
		obj.confidence = confidences[idx];
		obj.pts[0] = cv::Point2f(landmarks[idx][0], landmarks[idx][1]);
		obj.pts[1] = cv::Point2f(landmarks[idx][2], landmarks[idx][3]);
		obj.pts[2] = cv::Point2f(landmarks[idx][4], landmarks[idx][5]);
		obj.pts[3] = cv::Point2f(landmarks[idx][6], landmarks[idx][7]);
		obj.pts[8] = cv::Point2f(landmarks[idx][8], landmarks[idx][9]);

		for (int i = 0; i < 5; i++)
		{
			obj.pts[i] = cv::Point2f(landmarks[idx][2 * i], landmarks[idx][2 * i + 1]);
			// std::cout << "[x,y]  " << obj.pts[i].x << " " << obj.pts[i].y << std::endl;
		}
		objects.emplace_back(obj);
	}
	std::cout << "[size]: " << objects.size() << std::endl;

	return 0;
}

std::vector<BuffObject> DetectorProcess::run(cv::Mat &img)
{
	std::vector<BuffObject> objects;
	const float Threshold = 0.7;
	std::vector<float *> NetOutput;
	cv::Mat ResizedImg;

	ResizedImg = PreProcessYolo(img, 416, 416);

	net->SetInput(ResizedImg);

	net->Forward();

	net->GetOutputData(NetOutput);

	PostProcessYoloV5n(img, Threshold, NetOutput, objects);

	if (objects.size() == 0)
	{
		// 画中心线
		using namespace cv;
		// cv::line(img, Point2f(img.size().width / 2, 0), Point2f(img.size().width / 2, img.size().height), {0,255,0}, 1);
		// cv::line(img, Point2f(0, img.size().height / 2), Point2f(img.size().width, img.size().height / 2), {0,255,0}, 1);
	}
	else
	{
		std::cout << "[size1] " << objects.size() << std::endl;
	}

	return objects;
}
}