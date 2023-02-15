#include "../../include/buff/buff.h"
int threshold_param = 50;
cv::Mat binary;
#define RED
bool BuffDector::run(cv::Mat &image ,BUFF buff)
{
    // 灰度and二值
    cv::Mat splited_image[3];
    split(image, splited_image);
#ifdef RED
    binary = splited_image[2] - 0.1*splited_image[1] - 0.5f*splited_image[0];
#else
    binary = splited_image[0] - 0.1*splited_image[1] - 0.5f*splited_image[2];
#endif
    threshold(binary, binary, 100, 255, cv::THRESH_BINARY);
    cv::Mat element;
#ifdef RED
    element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
#else
    element = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
#endif
    morphologyEx(binary, binary, cv::MORPH_CLOSE, element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<double> areas;
    for(auto contour : contours)
    {
        double area = cv::contourArea(contour);
        areas.emplace_back(area);
    }

    double max_contour_area = argmax(areas);
    std::vector<cv::RotatedRect> rects;
    std::vector<cv::RotatedRect> fans;
    std::vector<cv::RotatedRect> flows;
    for(unsigned long i = 0; i < contours.size(); i++)
    {
        if(cv::contourArea(contours[i]) < max_contour_area/30)
        {
            continue;
        }
        if(hierarchy[i][2] == -1 && hierarchy[i][3] == -1)
        {
            //cv::drawContours(image, contours, i, cv::Scalar(0,0,255), 1);
            cv::RotatedRect rect;
            rect = cv::minAreaRect(contours[i]);
            cv::Point2f vertices[4];
            rect.points(vertices);
            rects.emplace_back(rect);
            if(judgeContourByConvexity(contours[i]))
            {
                flows.emplace_back(rect);
                //continue;
            }else
            {
                fans.emplace_back(rect);
            }
            for (int j = 0; j < 4; j++)
            {
                //cv::line(image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),1);
            }
        }
    }

    if(flows.size() != 1 && fans.size() > 10)
    {
        std::cout << "this frame failed! " << std::endl;
        return false;
    }
    std::vector<float> distances;
    for(auto fan : fans)
    {
        float distance = std::sqrt(std::pow(fan.center.x - flows[0].center.x,2)+std::pow(fan.center.y - flows[0].center.y,2));
        distances.emplace_back(distance);
    }

    int index = argMinIndex(distances);
    cv::Point2f vertices_fan[4];
    fans[index].points(vertices_fan);

    cv::Point2f vertices_flow[4];
    flows[0].points(vertices_flow);

    for (int j = 0; j < 4; j++)
    {
        //cv::line(image, vertices_fan[j], vertices_fan[(j + 1) % 4], cv::Scalar(0, 255, 0),1);
        //cv::line(image, vertices_flow[j], vertices_flow[(j + 1) % 4], cv::Scalar(0, 255, 0),1);
    }

//    cv::circle(image,vertices_flow[1],5,cv::Scalar(0,255,0),cv::FILLED);   // 绿色
//    cv::circle(image,vertices_flow[0],5,cv::Scalar(18,153,255),cv::FILLED);// 镉黄
//    cv::circle(image,vertices_fan[3],5,cv::Scalar(211,160,211),cv::FILLED);// 梅红色
//    cv::circle(image,vertices_fan[2],5,cv::Scalar(0,0,0),cv::FILLED);      // 黑

//    cv::circle(image,vertices_flow[0],12,cv::Scalar(0,255,0),cv::FILLED);
//    cv::circle(image,vertices_flow[1],12,cv::Scalar(18,153,255),cv::FILLED);
//    cv::circle(image,vertices_flow[2],12,cv::Scalar(211,160,211),cv::FILLED);
//    cv::circle(image,vertices_flow[3],12,cv::Scalar(0,0,0),cv::FILLED);


//    cv::circle(image,vertices_fan[0],12,cv::Scalar(0,255,0),cv::FILLED);
//    cv::circle(image,vertices_fan[1],12,cv::Scalar(18,153,255),cv::FILLED);
//    cv::circle(image,vertices_fan[2],12,cv::Scalar(211,160,211),cv::FILLED);
//    cv::circle(image,vertices_fan[3],12,cv::Scalar(0,0,0),cv::FILLED);

    cv::Point2f vector_of_fan_and_flow = fans[index].center - flows[0].center;
    float angle = 0;
    vector_of_fan_and_flow.y = -vector_of_fan_and_flow.y;
    //std::cout << "x " << vector_of_fan_and_flow.x << "  " << "y " << vector_of_fan_and_flow.y << std::endl;
    if(vector_of_fan_and_flow.x >= 0 )
    {
        angle = std::atan2(vector_of_fan_and_flow.x,vector_of_fan_and_flow.y)*(180/CV_PI);
    }else
    {
        angle = std::atan2(vector_of_fan_and_flow.x,vector_of_fan_and_flow.y)*(180/CV_PI) + 360;
        std::cout << "angle " << angle << std::endl;
    }

    if(angle == 0)
    {
        angle = 360;
    }

    if(angle > 0 && angle <= 90)
    {
        std::cout << "0---90" << std::endl;
        buff.fan[0] = vertices_fan[0];
        buff.fan[1] = vertices_fan[3];
        buff.fan[2] = vertices_flow[2];
        buff.fan[3] = vertices_flow[1];
    }else if (angle > 90 && angle <= 180)
    {
        std::cout << "90---180" << std::endl;
        buff.fan[0] = vertices_fan[1];
        buff.fan[1] = vertices_fan[0];
        buff.fan[2] = vertices_flow[3];
        buff.fan[3] = vertices_flow[2];
    }else if(angle > 180 && angle <= 270)
    {
        std::cout << "180---270" << std::endl;
        buff.fan[0] = vertices_fan[2];
        buff.fan[1] = vertices_fan[1];
        buff.fan[2] = vertices_flow[0];
        buff.fan[3] = vertices_flow[3];
    }else if(angle > 270 && angle <= 360)
    {
        std::cout << "270---360" << std::endl;
        buff.fan[0] = vertices_fan[3];
        buff.fan[1] = vertices_fan[2];
        buff.fan[2] = vertices_flow[1];
        buff.fan[3] = vertices_flow[0];
    }

    cv::circle(image,buff.fan[0],3,cv::Scalar(0,255,0),cv::FILLED);
    cv::circle(image,buff.fan[1],3,cv::Scalar(18,153,255),cv::FILLED);
    cv::circle(image,buff.fan[2],3,cv::Scalar(211,160,211),cv::FILLED);
    cv::circle(image,buff.fan[3],3,cv::Scalar(255,255,0),cv::FILLED);

    //cv::imshow("binary",binary);
    //cv::imshow("imshow",image);
    return true;
}

bool BuffDector::pointInRotatedRect(cv::RotatedRect a, cv::Point2f b)
{
    cv::Point2f vertices[4];
    a.points(vertices);// The order is bottomLeft, topLeft, topRight, bottomRight
    // transform
    vertices[0].y=-vertices[0].y;
    vertices[1].y=-vertices[1].y;
    vertices[2].y=-vertices[2].y;
    vertices[3].y=-vertices[3].y;
    b.y=-b.y;

    return (((b.x-vertices[1].x)*(vertices[0].x-vertices[1].x)+(b.y-vertices[1].y)*(vertices[0].y-vertices[1].y))
                    /(std::sqrt((b.x-vertices[1].x)*(b.x-vertices[1].x)+(b.y-vertices[1].y)*(b.y-vertices[1].y))*std::sqrt((vertices[0].x-vertices[1].x)*(vertices[0].x-vertices[1].x)+(vertices[0].y-vertices[1].y)*(vertices[0].y-vertices[1].y))))

          *(((b.x-vertices[1].x)*(vertices[2].x-vertices[1].x)+(b.y-vertices[1].y)*(vertices[2].y-vertices[1].y))
                    /(std::sqrt((b.x-vertices[1].x)*(b.x-vertices[1].x)+(b.y-vertices[1].y)*(b.y-vertices[1].y))*std::sqrt((vertices[2].x-vertices[1].x)*(vertices[2].x-vertices[1].x)+(vertices[2].y-vertices[1].y)*(vertices[2].y-vertices[1].y))))

          *(((b.x-vertices[3].x)*(vertices[2].x-vertices[3].x)+(b.y-vertices[3].y)*(vertices[2].y-vertices[3].y))
                    /(std::sqrt((b.x-vertices[3].x)*(b.x-vertices[3].x)+(b.y-vertices[3].y)*(b.y-vertices[3].y))*std::sqrt((vertices[2].x-vertices[3].x)*(vertices[2].x-vertices[3].x)+(vertices[2].y-vertices[3].y)*(vertices[2].y-vertices[3].y))))

          *(((b.x-vertices[3].x)*(vertices[0].x-vertices[3].x)+(b.y-vertices[3].y)*(vertices[0].y-vertices[3].y))
                    /(std::sqrt((b.x-vertices[3].x)*(b.x-vertices[3].x)+(b.y-vertices[3].y)*(b.y-vertices[3].y))*std::sqrt((vertices[0].x-vertices[3].x)*(vertices[0].x-vertices[3].x)+(vertices[0].y-vertices[3].y)*(vertices[0].y-vertices[3].y))));

}

bool BuffDector::judgeContourByConvexity(const std::vector<cv::Point> &contour)
{
    double hull_area, contour_area;

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);

    hull_area = cv::contourArea(hull);
    contour_area = cv::contourArea(contour);

    if (hull_area/contour_area > 3.0f)  // 判断凹凸性
        return true;

    return false;
}













