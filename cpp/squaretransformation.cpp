#include "squaretransformation.h"


cv::Rect SquareTransformation::getBrect(){
    return brect;
}

cv::Mat SquareTransformation::getT(){
    return T;
}

cv::Mat SquareTransformation::transform(std::vector<cv::Point> cornerPoints) {
    cv::Rect brect = cv::boundingRect(cornerPoints);
    int size = std::min(brect.width, brect.height);
    this->brect = cv::Rect(brect.x, brect.y, size, size);

    std::vector<cv::Point2f> corners = {
        cv::Point{brect.x, brect.y},
        {size + brect.x, brect.y},        // top right
        {size + brect.x, size + brect.y}, // bottom right
        {brect.x, size + brect.y},        // bottom left
    };

    std::vector<cv::Point2f> test = {
        cv::Point2f{cornerPoints[1].x, cornerPoints[1].y},
        {cornerPoints[0].x, cornerPoints[0].y}, // top right
        {cornerPoints[3].x, cornerPoints[3].y}, // bottom right
        {cornerPoints[2].x, cornerPoints[2].y}, // bottom left
    };

    T = cv::getPerspectiveTransform(test, corners);
    return T;
}