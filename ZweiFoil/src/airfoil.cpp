#include "ZweiFoil/airfoil.hpp"
#include <iostream>

namespace zweifoil {

Airfoil::Airfoil() : name("unknown") {}

bool Airfoil::loadFromFile(const std::string& filename) {

    std::cout << "Loading airfoil coords from: " << filename << "...\n";

    name = "NACA 0012 (Test)";

    return true;
}
}