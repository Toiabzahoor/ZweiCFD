#pragma once

#include <vector>
#include <string>

namespace zweifoil {

struct Point2D {
    double x;
    double y;
};

class Airfoil {
public:
    Airfoil();

    //loading airfoil coords from file
    bool loadFromFile(const std::string& filename);

    const std::vector<Point2D>& getCoordinates() const { return coordinates; }
    std::string getName() const { return name; }

private:
    std::string name;
    std::vector<Point2D> coordinates;
};
}