#ifndef BOARD
#define BOARD

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#include <Perception/opencv.h>
#include <math.h>

class Board {
  public:
    Board();
    Board(arr cent, double rad);
    void updateCorners(arr corners);
    arr getCellPosition(int i);
    arr getCenter();
    double getRadius();
    arr getY();
    std::vector<cv::Point> getCornerImgPoints();
    void setCornerImgPoints(std::vector<cv::Point> cornerPoints);

    cv::Mat region;
    int updated;

  private:
    arr corners;
    double radius;
    arr center;
    std::vector<cv::Point> cornerImgPoints;
};

#endif
