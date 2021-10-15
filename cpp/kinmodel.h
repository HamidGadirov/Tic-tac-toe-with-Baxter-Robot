#ifndef KINMODEL
#define KINMODEL 1
#ifndef ROSCOM_BAXTER
#define ROSCOM_BAXTER 1
#include <RosCom/baxter.h>
#endif

#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Kin/kin.h>
#include <Operate/robotOperation.h>

class KinModel {
  public:
    rai::KinematicWorld K;
    RobotOperation B;

    static KinModel &instance() {
        static KinModel _instance;
        return _instance;
    }

    void sync();

    ~KinModel() = default;

  private:
    KinModel() : K("./model.g"), B(K, 0.01, "rosnodetictactoe"){};

    KinModel(const KinModel &) = default;

    KinModel &operator=(const KinModel &) = default;
};

#endif
