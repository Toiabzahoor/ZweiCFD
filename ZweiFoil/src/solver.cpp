#include "ZweiFoil/solver.hpp"
#include <iostream>

namespace zweifoil {

Solver::Solver(const Airfoil& airfoil) : targetAirfoil(airfoil) {}

Coefficients Solver::runInviscid(const Flowconditions& conditions) {
    std::cout << "Running inviscid solver at alpha = " << conditions.alpha << " degrees...\n";

    //test coeffs for now
    Coefficients coeffs = {0.0,0.0,0.0};
    return coeffs;
}
}