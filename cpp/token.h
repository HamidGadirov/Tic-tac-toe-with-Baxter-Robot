#ifndef TOKEN_H
#define TOKEN_H

#include <Core/array.h>
#include "kinmodel.h"


class Token {
    public:
    arr center;
    double valid = 0.;

    arr getY();
    arr getOrientation();
    void updateCorners(arr corners);
    

    private:
    arr corners;
    rai::Frame *tokennn = KinModel::instance().K.addFrame("tokennn");
    
};

#endif
