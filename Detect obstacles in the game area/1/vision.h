#ifndef VISION
#define VISION

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#include <Perception/opencv.h>
#include <Core/thread.h>
#include <RosCom/rosCamera.h>
#include <RosCom/roscom.h>
#include <Perception/depth2PointCloud.h>
#include "kinmodel.h"
#include "board.h"

class Vision : public Thread {

  public:

    Vision();
    cv::Mat selectColor(cv::Mat rgb_img, int selector);
    std::vector<cv::Point> findBiggestTetragon(cv::Mat binary_img);
    std::vector<cv::Point>
    findBiggestContourArea(std::vector<std::vector<cv::Point>> contours);
    cv::Mat squareTransformation(std::vector<cv::Point> cornerPoints,
                                 cv::Mat input_img);
    float medianFilter(cv::Mat depth_img);
    int detectToken(int cellnum);
    int detectNextMove();

    void open();
    void step();
    void close();

    Board getBoard();

  private:
  
    arr Fxypxy;
    Var<byteA> _rgb;
    Var<floatA> _depth;
    Depth2PointCloud *d2p;
    rai::Frame *cameraFrame;
    rai::Frame *ball;
    RosCamera *cam;
    Board board;
    rai::Frame *playboard;
    rai::KinematicWorld &K;
};

#endif
