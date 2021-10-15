#ifndef KINEMATIC
#define KINEMATIC

#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#ifndef RAI_ROS
#define RAI_ROS 1
#endif

#include <Perception/opencv.h>
#include "kinmodel.h"
#include <Core/thread.h>
#include <Kin/kin.h>
#include <math.h>

class Kinematic : public Thread {
  public:
    Kinematic();
    arr getJointState(arr position, arr q_0, arr ydir);
    void placeToken(arr position);
    bool stopRobot();
    void goHome();
    void goToJointState(arr q_home);
    void toggleVacuumGripper();
    void liftEndeffector(double dist, arr ydir);

    void open();
    void step();
    void close();

  private:
    RobotOperation &B;
};

#endif
