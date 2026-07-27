#include "ZweiFoil/airfoil.hpp"
#include <iostream>
#include <cmath>


namespace zweifoil {

Airfoil::Airfoil() : name("unknown") {}

bool Airfoil::loadFromFile(const std::string& filename) {

    std::cout << "Loading airfoil coords from: " << filename << "...\n";

    name = "NACA 0012 (Test)";

    coordinates = {
        Point2D{1.0, 0.0}, 
        Point2D{0.5, 0.1}, 
        Point2D{0.0, 0.0}, 
        Point2D{0.5, -0.1}, 
        Point2D{1.0, 0.0}
    };
    
    generatePanels();
    return true;
}

void Airfoil::generatePanels() {
    panels.clear();
    if (coordinates.size() < 2) return;

    for (size_t i = 0; i < coordinates.size() - 1; ++i) {
        Panel p;
        p.p1 = coordinates[i];
        p.p2 = coordinates[i + 1];

        double dx = p.p2.x - p.p1.x;
        double dy = p.p2.y - p.p1.y;

        //collocation point (midpoint)
        p.cp.x = (p.p1.x + p.p2.x) / 2.0;
        p.cp.y = (p.p1.y + p.p2.y) / 2.0;

        //length & angle
        p.length = std::sqrt(dx * dx + dy * dy);
        p.theta = std::atan2(dy, dx);

        //tangent vector
        p.tangent.x = std::cos(p.theta);
        p.tangent.y = std::sin(p.theta);

        //normal vector
        p.normal.x = -std::sin(p.theta);
        p.normal.y = std::cos(p.theta);

        panels.push_back(p);
    }

    std::cout << "Generated " << panels.size() << " panels for airfoil: " << name << "\n";

}
}