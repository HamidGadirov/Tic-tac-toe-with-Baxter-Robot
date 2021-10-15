#include "cell.h"

#include <cassert>

arr arrFromGrid(Grid g) {
    arr a;

    for (Cell c : g) {
        a.append((double)c);
    }

    return a;
}

Cell cellFromDouble(double d) {
    switch ((int)d) {
    case 0:
        return Cell::Empty;
    case 1:
        return Cell::Robot;
    case 2:
        return Cell::Player;
    default:
        return Cell::Invalid;
    }
}

Grid gridFromArr(arr a) {
    assert(a.nd == 1);
    assert(a.d0 == 9);

    Grid g;
    for (int i = 0; i < 9; ++i) {
        g[i] = cellFromDouble(a(i));
    }

    return g;
}

std::ostream &operator<<(std::ostream &os, Cell c) {
    switch (c) {
    case Cell::Empty:
        return os << ' ';
    case Cell::Robot:
        return os << 'X';
    case Cell::Player:
        return os << 'O';
    case Cell::Invalid:
        return os << '-';
    }
}

std::ostream &operator<<(std::ostream &os, Grid g) {
    for (Cell c : g) {
        os << c;
    }

    return os;
}
