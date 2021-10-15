#ifndef SQUARETRANSFORMATION
#define SQUARETRANSFORMATION

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif

#include <Perception/opencv.h>

class SquareTransformation {
    public:
    cv::Mat transform(std::vector<cv::Point> cornerPoints);
    cv::Rect getBrect();
    cv::Mat getT();

    private:
    cv::Rect brect;
    cv::Mat T;
};

#endif
