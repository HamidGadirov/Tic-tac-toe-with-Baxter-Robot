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
    arr getJointState(arr position, arr q_0, arr ori=arr(3,3,{0,0,0,0,0,0,0,0,0}));
    void placeToken(arr position);
    bool stopRobot();
    void goHome(double time= 7.);
    void goCheer(double time= 7.);
    void goToJointState(arr q_home, double time=7.);
    void toggleVacuumGripper();
    void liftEndeffector(double dist, arr ori=arr(3,3,{0,0,0,0,0,0,0,0,0}),double time=7.);
    void goCheat(double time);
    void open();
    void step();
    void close();

  private:
    RobotOperation &B;
};

#endif
