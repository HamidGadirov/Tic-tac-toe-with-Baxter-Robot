#ifndef VISION
#define VISION

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif

#include <Perception/opencv.h>

#include "board.h"
#include "cell.h"
#include "token.h"
#include "squaretransformation.h"

//#include <Perception/opencv.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/rosCamera.h>
#include <Core/thread.h>

class Vision : public Thread {
  public:
    Vision();

    float medianFilter(cv::Mat img);

    Grid detectTokens();
    bool detectNextMove(Grid grid);
    arr calcCornerDepth(cv::Mat depth_img);

    void open();
    void step();
    void close();

    Board getBoard();
    Token getToken();

    ~Vision() {
        threadClose();
    }

  private:
    cv::Mat squareTransformation(std::vector<cv::Point> cornerPoints,
                                 cv::Mat input_img);

    Cell detectTkn(int cellnum);

    Var<byteA> _rgb;
    Var<floatA> _depth;
    RosCamera cam1;

    Board board;
    Token token;

    cv::Mat rgb;
    cv::Mat T;
    cv::Rect brect;

    arr Pinv;
    SquareTransformation boardTransformation;
    SquareTransformation tokenTransformation;
};

#endif
