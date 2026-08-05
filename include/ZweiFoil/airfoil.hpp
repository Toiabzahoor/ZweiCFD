#pragma once

#include <vector>
#include <string>

namespace zweifoil {

struct Point2D {
    double x;
    double y;
};
struct Panel {
    Point2D p1; //start point
    Point2D p2; //end point
    Point2D cp; //collocation pt
    double length;
    double theta;
    Point2D normal;
    Point2D tangent;
};
class Airfoil {
public:
    Airfoil();
    //loading airfoil coords from file (not done properly yet).
    
    bool loadFromFile(const std::string& filename);

    void generatePanels(); //generate panels from coordinates
    void rotateCoordinates(double angleDeg);

    const std::vector<Point2D>& getCoordinates() const { return coordinates; }
    const std::vector<Panel>& getPanels() const { return panels; }

    std::string getName() const { return name; }

private:
    std::string name;
    std::vector<Point2D> coordinates;
    std::vector<Panel> panels; //storing generated panels
};
}