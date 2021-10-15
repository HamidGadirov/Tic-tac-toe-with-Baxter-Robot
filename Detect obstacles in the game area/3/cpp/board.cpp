#include "board.h"

arr Board::getCenter() {
    return center;
}

double Board::getRadius() {
    return radius;
}

arr Board::getCellPosition(int i) {
    arr topLeft = corners[0];
    arr topRight = corners[1];
    arr bottomLeft = corners[3];

    arr x_vector = topRight - topLeft;
    arr y_vector = bottomLeft - topLeft;

    arr x_third = x_vector / 3.;
    arr y_third = y_vector / 3.;

    int row = i / 3;
    int column = i % 3;
    arr x_pos = (double)column * x_third + x_third / 2.;
    arr y_pos = (double)row * y_third + y_third / 2.;

    return topLeft + x_pos + y_pos;
}

void Board::updateCorners(arr c) {
    ++updated;
    corners = c;
}

Board::Board() : Board(NoArr, 0.0) {}

Board::Board(arr cent, double rad) {
    center = cent;
    radius = rad;
    updated = 0;
    region.cols = 0;
    region.rows = 0;
}

arr Board::getY() {
    arr topLeft = corners[0];
    arr bottomLeft = corners[3];
    arr res = bottomLeft - topLeft;
    res = res / sqrt((~(res) * (res))(0));
    return res;
}

std::vector<cv::Point> Board::getCornerImgPoints() {
    return cornerImgPoints;
}

void Board::setCornerImgPoints(std::vector<cv::Point> cornerPoints) {
    cornerImgPoints = cornerPoints;
}
