#ifndef CONTROLLER
#define CONTROLLER

#include "kinematic.h"
#include "gamelogic.h"
#include "vision.h"

class Controller {
  public:
    Controller();
    void startController();
    void graspToken();

  private:
    Vision vision;
    Kinematic kinematic;
    Gamelogic gamelogic;
};

#endif
