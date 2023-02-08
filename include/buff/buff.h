#include "../common.h"

struct BUFF
{
    cv::Point2f fan[4];
    cv::Point3f armor;
    cv::Point3f center;
};

class BuffDector
{
public:
    BuffDector(){};
    ~BuffDector(){};

public:
    bool run(cv::Mat &img,BUFF buff);
private:
    // judge point in RotateRect
    bool pointInRotatedRect(cv::RotatedRect a, cv::Point2f b);
    // get max value
    template <class T>
    static inline int argmax(const std::vector<T> ptr)
    {
        int len = ptr.size();
        int max_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] > ptr[max_arg]) max_arg = i;
        }
        return ptr[max_arg];
    }

    template <class T>
    static inline int argMinIndex(const std::vector<T> ptr)
    {
        int len = ptr.size();
        int min_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] < ptr[min_arg]) min_arg = i;
        }
        return min_arg;
    }
    // judge contour convexity
    bool judgeContourByConvexity(const std::vector<cv::Point> &contour);


};
