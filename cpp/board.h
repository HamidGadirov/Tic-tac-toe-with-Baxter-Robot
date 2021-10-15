#ifndef BOARD
#define BOARD

#include "kinmodel.h"

class Board {
  public:
    void updateCorners(arr corners);
    arr getCellPosition(int i);
    arr getY();
    arr getOrientation();

    int updated = 0;

  private:
    arr corners;

    rai::Frame *playboard = KinModel::instance().K.addFrame("playboard");
};

#endif
