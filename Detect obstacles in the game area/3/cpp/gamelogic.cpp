#include "gamelogic.h"

arr Gamelogic::getLine(int i) {
    switch (i) {
    case 0:
        return {0., 1., 2.};
    case 1:
        return {3., 4., 5.};
    case 2:
        return {6., 7., 8.};
    case 3:
        return {0., 3., 6.};
    case 4:
        return {1., 4., 7.};
    case 5:
        return {2., 5., 8.};
    case 6:
        return {0., 4., 8.};
    case 7:
        return {2., 4., 6.};
    default:
        return {-1., -1., -1.};
    }
}

arr Gamelogic::getIncludedLines(int idx) {
    arr vect;
    arr buff;
    uint columns = 3;
    uint rows = 0;

    for (int i = 0; i < 8; i++) {
        buff = getLine(i);
        if (buff.contains(idx)) {
            vect.append(buff);
            rows++;
        }
    }
    uintA shape = {rows, columns};
    return vect.reshape(shape);
}

int Gamelogic::win(arr state) {
    for (int i = 0; i < 8; i++) {
        arr idx = getLine(i);
        if (state(idx(0)) == 2. || state(idx(1)) == 2. || state(idx(2)) == 2.) {
            continue;
        }
        if (state(idx(0)) == 0. && state(idx(1)) == 1. && state(idx(2)) == 1.) {
            return (int)idx(0);
        }
        if (state(idx(0)) == 1. && state(idx(1)) == 0. && state(idx(2)) == 1.) {
            return (int)idx(1);
        }
        if (state(idx(0)) == 1. && state(idx(1)) == 1. && state(idx(2)) == 0.) {
            return (int)idx(2);
        }
    }
    return -1;
}

int Gamelogic::block(arr state) {
    for (int i = 0; i < 8; i++) {
        arr idx = getLine(i);
        if (state(idx(0)) == 1. || state(idx(1)) == 1. || state(idx(2)) == 1.) {
            continue;
        }
        if (state(idx(0)) == 0. && state(idx(1)) == 2. && state(idx(2)) == 2.) {
            return (int)idx(0);
        }
        if (state(idx(0)) == 2. && state(idx(1)) == 0. && state(idx(2)) == 2.) {
            return (int)idx(1);
        }
        if (state(idx(0)) == 2. && state(idx(1)) == 2. && state(idx(2)) == 0.) {
            return (int)idx(2);
        }
    }
    return -1;
}

bool Gamelogic::forkOption(arr idx, arr state, double player) {
    if (state(idx(0)) == player && state(idx(1)) == 0. && state(idx(2)) == 0.) {
        return true;
    }
    if (state(idx(0)) == 0. && state(idx(1)) == player && state(idx(2)) == 0.) {
        return true;
    }
    if (state(idx(0)) == 0. && state(idx(1)) == 0. && state(idx(2)) == player) {
        return true;
    }
    return false;
}

int Gamelogic::fork(arr state) {
    for (int i = 0; i < 9; i++) {
        if (state(i) != 0.) {
            // No fork at position i possible
            continue;
        }
        int fork_possible = 0;
        arr lines = getIncludedLines(i);
        uint lines_n = lines.dim(0);
        for (int p = 0; p < lines_n; p++) {
            if (forkOption(lines[p], state, 1.)) {
                fork_possible++;
            }
            if (fork_possible == 2) {
                return i;
            }
        }
    }
    return -1;
}

arr Gamelogic::attackPositions(arr state, double player) {
    arr ret;
    for (int i = 0; i < 8; i++) {
        arr idx = getLine(i);
        if (state(idx(0)) == 0. && state(idx(1)) == player &&
            state(idx(2)) == 0.) {
            if (!ret.contains(idx(0))) {
                ret.append({idx(0)});
            }
            if (!ret.contains(idx(2))) {
                ret.append({idx(2)});
            }
        }

        if (state(idx(0)) == player && state(idx(1)) == 0. &&
            state(idx(2)) == 0.) {
            if (!ret.contains(idx(1))) {
                ret.append({idx(1)});
            }
            if (!ret.contains(idx(2))) {
                ret.append({idx(2)});
            }
        }

        if (state(idx(0)) == 0. && state(idx(1)) == 0. &&
            state(idx(2)) == player) {
            if (!ret.contains(idx(0))) {
                ret.append({idx(0)});
            }
            if (!ret.contains(idx(1))) {
                ret.append({idx(1)});
            }
        }
    }
    return ret;
}

int Gamelogic::blockingFork(arr state) {
    arr fork_positions;
    for (int i = 0; i < 9; i++) {
        if (state(i) != 0.) {
            // No fork at position i possible
            continue;
        }
        int fork_possible = 0;
        arr lines = getIncludedLines(i);
        uint lines_n = lines.dim(0);
        for (int p = 0; p < lines_n; p++) {
            if (forkOption(lines[p], state, 2.)) {
                fork_possible++;
            }
            if (fork_possible == 2) {
                fork_positions.append({(double)i});
                break;
            }
        }
    }
    if (fork_positions.empty()) {
        return -1;
    }
    if (fork_positions.dim(0) == 1) {
        return fork_positions(0);
    }
    if (fork_positions.dim(0) > 1) {
        arr buff = attackPositions(state, 1.);
        for (int i = 0; i < buff.dim(0); i++) {
            if (!fork_positions.contains(buff(i))) {
                return buff(i);
            }
        }
    }
    return -1;
}

int Gamelogic::center(arr state) {
    if (state(4) == 0.) {
        return 4;
    }
    return -1;
}

int Gamelogic::oppositeCorner(arr state) {
    if (state(0) == 2. && state(8) == 0.) {
        return 8;
    }
    if (state(0) == 0. && state(8) == 2.) {
        return 0;
    }
    if (state(2) == 2. && state(6) == 0.) {
        return 6;
    }
    if (state(2) == 0. && state(6) == 2.) {
        return 2;
    }
    return -1;
}

int Gamelogic::emptyCorner(arr state) {
    if (state(0) == 0.) {
        return 0;
    }
    if (state(2) == 0.) {
        return 2;
    }
    if (state(6) == 0.) {
        return 6;
    }
    if (state(8) == 0.) {
        return 8;
    }
    return -1;
}

int Gamelogic::emptySide(arr state) {
    if (state(1) == 0.) {
        return 1;
    }
    if (state(3) == 0.) {
        return 3;
    }
    if (state(5) == 0.) {
        return 5;
    }
    if (state(7) == 0.) {
        return 7;
    }
    return -1;
}

int Gamelogic::getNextMove(arr state) {
    /*
    -1 invalid
    0 free
    1 my token
    2 opponents token
     */

    int ret = -1;

    // 1. Win:
    ret = win(state);
    if (ret != -1) {
        cout << "Win Move" << endl;
        gamestate = 1;
        return ret;
    }

    // 2. Block:
    ret = block(state);
    if (ret != -1) {
        cout << "Block Move" << endl;
        return ret;
    }

    // 3. Fork:
    ret = fork(state);
    if (ret != -1) {
        cout << "Fork Move" << endl;
        return ret;
    }

    // 4. Blocking Fork:
    ret = blockingFork(state);
    if (ret != -1) {
        cout << "Blocking Fork Move" << endl;
        return ret;
    }

    // 5. Center:
    ret = center(state);
    if (ret != -1) {
        cout << "Center Move" << endl;
        return ret;
    }

    // 6. Opposite Corner:
    ret = oppositeCorner(state);
    if (ret != -1) {
        cout << "Opposite Corner Move" << endl;
        return ret;
    }

    // 7. Empty Corner:
    ret = emptyCorner(state);
    if (ret != -1) {
        cout << "Empty Corner Move" << endl;
        return ret;
    }

    // 8. Empty Side:
    ret = emptySide(state);
    if (ret != -1) {
        cout << "Empty Side Move" << endl;
        return ret;
    }

    gamestate = 2;

    return ret;
}

int Gamelogic::getGamestate(void) {
    return gamestate;
}