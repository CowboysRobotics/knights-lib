#include "knights/autonomous/path.h"

#include "knights/util/position.h"

knights::Route::Route(std::vector<Pos> positions) {
    this->positions = positions;
}

knights::Route::Route() {
    this->positions = {};
}