#ifndef GAME_LOGIC
#define GAME_LOGIC

#include <Core/array.h>

class Gamelogic {
  public:
    int getNextMove(arr state);
    int getGamestate(void);

  private:
    arr getLine(int i);
    arr getIncludedLines(int idx);
    int win(arr state);
    int block(arr state);
    bool forkOption(arr idx, arr state, double player);
    int fork(arr state);
    arr attackPositions(arr state, double player);
    int blockingFork(arr state);
    int center(arr state);
    int oppositeCorner(arr state);
    int emptyCorner(arr state);
    int emptySide(arr state);

    int gamestate = 0; // 0 running, 1 robot won, 2 draw
};

#endif
