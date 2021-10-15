#include "board.h"

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
    /*arr buff1 = c.copy();
    arr buff2 = c.copy();

    arr tl = buff1[0];
    arr tr = buff1[1];
    arr br = buff1[2];
    arr bl = buff1[3];

    arr vec1 = br - tl;
    arr vec2 = bl - tr;

    arr cp = {  vec1(1)*vec2(2) - vec1(2)*vec2(1), 
                vec1(2)*vec2(0) - vec1(0)*vec2(2), 
                vec1(0)*vec2(1) - vec1(1)*vec2(0)};

    cp = (1/(sqrt((~cp*cp)(0)))*0.02)*cp;

    buff2[0] += cp;
    buff2[1] += cp;
    buff2[2] += cp;
    buff2[3] += cp;

    buff1.reshape(12);
    buff2.reshape(12);
    buff1.append(buff2);
    std::vector<double> boardshape;
    for (int i = 0; i < 24; i++) {
        boardshape.push_back(buff1.elem(i));
    }
    //playboard->setConvexMesh(boardshape, {0, 255, 0});
    KinModel::instance().K.watch();*/
}

arr Board::getY() {
    arr tl = corners[0];
    arr bl = corners[3];
    arr tr = corners[1];
    arr br = corners[2];
    arr res = ((bl - tl)+(br-tr))/2.;
    res = res / sqrt((~(res) * (res))(0));
    return res;
}


arr Board::getOrientation() {
    arr tl = corners[0];
    arr bl = corners[3];
    arr tr = corners[1];
    arr br = corners[2];

    arr x_dir = ((tr - tl)+(bl-bl))/2.;
    x_dir = x_dir / sqrt((~(x_dir) * (x_dir))(0));

    arr y_dir = ((bl - tl)+(br-tr))/2.;
    y_dir = y_dir / sqrt((~(y_dir) * (y_dir))(0));

    arr z_dir = {0,0,0};



    x_dir.append(y_dir);
    x_dir.append(z_dir);

    arr res = x_dir.reshape(3,3);
    return res;
}