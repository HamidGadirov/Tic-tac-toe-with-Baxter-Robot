#include "token.h"

arr Token::getOrientation() {
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

arr Token::getY() {
    arr tl = corners[0];
    arr bl = corners[3];
    arr tr = corners[1];
    arr br = corners[2];
    arr res = ((bl - tl)+(br-tr))/2.;
    res = res / sqrt((~(res) * (res))(0));
    return res;
}

void Token::updateCorners(arr c) {
    corners = c;
    arr buff = c.copy();

    //cout << "Corner Coords of token: " << c << endl;
    /*
    buff.reshape(12);
    std::vector<double> boardshape;
    for (int i = 0; i < 12; i++) {
        boardshape.push_back(buff.elem(i));
    }
    tokennn->setConvexMesh(boardshape, {255, 0, 0});*/
    //KinModel::instance().K.watch();
}