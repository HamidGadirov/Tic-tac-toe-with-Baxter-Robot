#ifndef CELL
#define CELL

#include <array>
#include <Core/array.h>

enum class Cell {
    Invalid = -1,
    Empty,
    Robot,
    Player,
};

Cell cellFromDouble(double d);

using Grid = std::array<Cell, 9>;

arr arrFromGrid(Grid g);
Grid gridFromArr(arr a);

std::ostream &operator<<(std::ostream &os, Cell c);
std::ostream &operator<<(std::ostream &os, Grid g);

#endif
