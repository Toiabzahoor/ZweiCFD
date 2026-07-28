#include "ZweiFoil/airfoil.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>


namespace zweifoil {

Airfoil::Airfoil() : name("unknown") {}

static void loadDummyDiamond(std::string& name, std::vector<Point2D>& coordinates) {
    name = "NACA 0012 (Test)";
    coordinates = {
        Point2D{1.0, 0.0},
        Point2D{0.5, 0.1},
        Point2D{0.0, 0.0},
        Point2D{0.5, -0.1},
        Point2D{1.0, 0.0}
    };
}

bool Airfoil::loadFromFile(const std::string& filename) {

    std::cout << "Loading airfoil coords from: " << filename << "...\n";

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "  Could not open '" << filename << "', using dummy airfoil instead.\n";
        loadDummyDiamond(name, coordinates);
        generatePanels();
        return true;
    }

    std::vector<Point2D> parsed;
    std::string line;
    bool firstLine = true;
    std::string parsedName;

    while (std::getline(file, line)) {
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        size_t end = line.find_last_not_of(" \t\r\n");
        std::string trimmed = line.substr(start, end - start + 1);

        std::istringstream iss(trimmed);
        double x, y;
        if (iss >> x >> y) {
            parsed.push_back(Point2D{x, y});
        } else if (firstLine) {
            parsedName = trimmed;
        }
        firstLine = false;
    }

    if (parsed.size() < 3) {
        std::cerr << "  File '" << filename << "' had too few coordinate points ("
                  << parsed.size() << "), using dummy airfoil instead.\n";
        loadDummyDiamond(name, coordinates);
        generatePanels();
        return true;
    }

    name = parsedName.empty() ? filename : parsedName;
    coordinates = parsed;

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