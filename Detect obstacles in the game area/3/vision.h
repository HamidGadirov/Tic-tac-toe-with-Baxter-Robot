#ifndef VISION
#define VISION

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#include "board.h"
#include "kinmodel.h"

#include <Core/thread.h>
#include <Perception/depth2PointCloud.h>
#include <Perception/opencv.h>
#include <RosCom/rosCamera.h>
#include <RosCom/roscom.h>
#include <algorithm>

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
    int detectTkn(int cellnum, Board brd);
    arr detectTokens(Board brd);
    bool detectNextMove();

    void open();
    void step();
    void close();

    void onlinetracking(cv::Mat input_img);
    arr globalPointToCameraPoint(arr pt);
    arr cameraPointToImgCoords(arr pt);
    std::vector<Board> findNNObjects(cv::Mat input_img, Board circle, int nn);

    Board getBoard();

    ~Vision() {threadClose();}

  private:
    arr Fxypxy;
    //Var<byteA> _rgb;
    Var<floatA> _depth;
    Depth2PointCloud *d2p;
    rai::Frame *cameraFrame;
    rai::Frame *ball;
    //RosCamera *cam;
    Board board;
    rai::Frame *playboard;
    rai::KinematicWorld &K;
    cv::Mat rgb;
    cv::Mat T;
    cv::Rect brect;
};

#endif
