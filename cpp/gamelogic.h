#ifndef GAME_LOGIC
#define GAME_LOGIC

#include <Core/array.h>
#include "cell.h"

enum class GameState {
    Running,
    RobotWon,
    PlayerWon,
    Draw,
};

class Gamelogic {
  public:
    int getNextMove(Grid state);
    GameState getGameState(Grid g);
    bool detectCheating(Grid oldGrid,Grid newGrid,Cell val);

  private:
    
};

#endif
