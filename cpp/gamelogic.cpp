#include "gamelogic.h"

#include "cell.h"

#include <algorithm>
#include <utility>

using LineIdx = std::array<int, 3>;
using Line = std::array<Cell, 3>;

const std::array<LineIdx, 8> lines = {{{0, 1, 2},
                                       {3, 4, 5},
                                       {6, 7, 8},
                                       {0, 3, 6},
                                       {1, 4, 7},
                                       {2, 5, 8},
                                       {0, 4, 8},
                                       {2, 4, 6}}};

Line extractLine(Grid g, LineIdx i) {
    return {g[i[0]], g[i[1]], g[i[2]]};
}

GameState Gamelogic::getGameState(Grid g) {
    Cell r = Cell::Robot;
    Cell p = Cell::Player;
    for (LineIdx idx : lines) {
        Line l = extractLine(g, idx);
        if (l == Line{r, r, r}) {
            return GameState::RobotWon;
        }
        if (l == Line{p, p, p}) {
            return GameState::PlayerWon;
        }
    }

    for (Cell c : g) {
        if (c == Cell::Empty)
            return GameState::Running;
    }

    return GameState::Draw;
}

std::vector<std::array<int,9>> getGridOrientation(){
    std::vector<std::array<int,9>> ret;
    ret.push_back({0,1,2,3,4,5,6,7,8});
    ret.push_back({6,3,0,7,4,1,8,5,2});
    ret.push_back({8,7,6,5,4,3,2,1,0});
    ret.push_back({2,5,8,1,4,7,0,3,6});
    return ret;

}


bool Gamelogic::detectCheating(Grid oldGrid,Grid newGrid,Cell val){
    //Returns true if cheated, false otherwise

    int cnt;
    int idx;

    auto buff = getGridOrientation();
    for (std::array<int,9> arr : buff){
        cnt = 0;
        for(int i = 0; i < 9; i++){
            if(oldGrid[i] == newGrid[arr[i]]){
                cnt ++;
            }
            else{
                idx = i;
            }
        }
        if(cnt == 9 && val == Cell::Empty){
            return false;
        }

        if(cnt == 8){
            if( val == Cell::Robot &&
                oldGrid[idx] == Cell::Empty && 
                newGrid[arr[idx]] == Cell::Robot){
                return false;
            }
            if( val == Cell::Player &&
                oldGrid[idx] == Cell::Empty && 
                newGrid[arr[idx]] == Cell::Player){
                return false;
            }
        }
    }
    return true;

    // Player should have one additional token

}


std::vector<LineIdx> getIncludedLines(int idx) {
    std::vector<LineIdx> vect{3};

    for (LineIdx buff : lines) {
        if (std::find(buff.begin(), buff.end(), idx) != buff.end()) {
            vect.push_back(buff);
        }
    }

    return vect;
}

int winningSpot(Grid g, Cell p) {
    for (LineIdx idx : lines) {
        Line l = extractLine(g, idx);

        if (l == Line{Cell::Empty, p, p}) {
            return idx[0];
        }

        if (l == Line{p, Cell::Empty, p}) {
            return idx[1];
        }

        if (l == Line{p, p, Cell::Empty}) {
            return idx[2];
        }
    }

    return -1;
}

int win(Grid g) {
    return winningSpot(g, Cell::Robot);
}

int block(Grid g) {
    return winningSpot(g, Cell::Player);
}

bool forkOption(LineIdx idx, Grid g, Cell p) {
    auto l = extractLine(g, idx);

    return l == Line{p, Cell::Empty, Cell::Empty} ||
           l == Line{Cell::Empty, p, Cell::Empty} ||
           l == Line{Cell::Empty, Cell::Empty, p};
}

bool forkPosition(Grid g, Cell player, int location) {
    unsigned fork_possible = 0;
    for (LineIdx line : getIncludedLines(location)) {
        if (forkOption(line, g, player)) {
            fork_possible++;
        }

        if (fork_possible == 2) {
            return true;
        }
    }

    return false;
}

std::vector<int> forkPositions(Grid g, Cell player) {
    std::vector<int> pos;
    for (int i = 0; i < 9; i++) {
        if (g[i] != Cell::Empty) {
            // No fork at position i possible
            continue;
        }

        if (forkPosition(g, player, i)) {
            pos.push_back(i);
        }
    }

    return pos;
}

int fork(Grid g) {
    auto pos = forkPositions(g, Cell::Robot);

    return pos.empty() ? -1 : pos.front();
}

std::vector<LineIdx> attackPositions(Grid g, Cell p) {
    std::vector<LineIdx> attackLines;

    for (LineIdx idx : lines) {
        if (forkOption(idx, g, p))
            attackLines.push_back(idx);
    }

    return attackLines;
}

int blockingFork(Grid g) {
    std::vector<int> fork_positions = forkPositions(g, Cell::Player);

    if (fork_positions.empty()) {
        return -1;
    }

    if (fork_positions.size() == 1) {
        return fork_positions.front();
    }

    auto buff = attackPositions(g, Cell::Robot);
    for (LineIdx idx : buff) {
        auto l = extractLine(g, idx);

        int i1, i2;
        if (l[0] == Cell::Robot) {
            i1 = idx[1];
            i2 = idx[2];
        }

        if (l[1] == Cell::Robot) {
            i1 = idx[0];
            i2 = idx[2];
        }

        if (l[2] == Cell::Robot) {
            i1 = idx[0];
            i2 = idx[1];
        }

        if (std::find(fork_positions.begin(), fork_positions.end(), i1) ==
            fork_positions.end()) {
            return i2;
        }

        if (std::find(fork_positions.begin(), fork_positions.end(), i2) ==
            fork_positions.end()) {
            return i1;
        }
    }

    return -1;
}

int center(Grid g) {
    return g[4] == Cell::Empty ? 4 : -1;
}

int oppositeCorner(Grid g) {
    for (auto p : {std::make_pair(0, 8), std::make_pair(2, 6)}) {
        if (g[p.first] == Cell::Player && g[p.second] == Cell::Empty) {
            return p.second;
        }

        if (g[p.second] == Cell::Player && g[p.first] == Cell::Empty) {
            return p.first;
        }
    }

    return -1;
}

int emptyCorner(Grid g) {
    for (int i : {0, 2, 6, 8}) {
        if (g[i] == Cell::Empty) {
            return i;
        }
    }

    return -1;
}

int emptySide(Grid g) {
    for (int i : {1, 3, 5, 7}) {
        if (g[i] == Cell::Empty) {
            return i;
        }
    }

    return -1;
}

int Gamelogic::getNextMove(Grid g) {
    int ret = -1;

    // 1. Win:
    ret = win(g);
    if (ret != -1) {
        cout << "Win Move" << endl;
        return ret;
    }

    // 2. Block:
    ret = block(g);
    if (ret != -1) {
        cout << "Block Move: " << ret << endl;
        return ret;
    }

    // 3. Fork:
    ret = fork(g);
    if (ret != -1) {
        cout << "Fork Move: " << ret << endl;
        return ret;
    }

    // 4. Blocking Fork:
    ret = blockingFork(g);
    if (ret != -1) {
        cout << "Blocking Fork Move: " << ret << endl;
        return ret;
    }

    // 5. Center:
    ret = center(g);
    if (ret != -1) {
        cout << "Center Move" << endl;
        return ret;
    }

    // 6. Opposite Corner:
    ret = oppositeCorner(g);
    if (ret != -1) {
        cout << "Opposite Corner Move" << endl;
        return ret;
    }

    // 7. Empty Corner:
    ret = emptyCorner(g);
    if (ret != -1) {
        cout << "Empty Corner Move" << endl;
        return ret;
    }

    // 8. Empty Side:
    ret = emptySide(g);
    if (ret != -1) {
        cout << "Empty Side Move" << endl;
        return ret;
    }

    return ret;
}

