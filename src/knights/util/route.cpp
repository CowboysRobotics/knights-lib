#include "knights/autonomous/path.h"

#include "knights/util/position.h"

knights::Route::Route(std::vector<Pos> positions) {
    this->positions = positions;
}

knights::Route::Route() {
    this->positions = {};
}

knights::Route knights::operator+(const Route &r1, const Route &r2) {
    return Route(r1.positions + r2.positions);
};

knights::Route knights::operator+(knights::Route r1, const knights::Pos &p1) {
    r1.positions.push_back(p1);
    return r1;
};

knights::Route knights::operator-(knights::Route r1, const int &amt) {
    r1.positions.resize(r1.positions.size()-std::min((int)r1.positions.size(), amt));
    return r1;
}