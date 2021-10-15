#include "kinmodel.h"

void KinModel::sync() {
    B.sync(K);
}
