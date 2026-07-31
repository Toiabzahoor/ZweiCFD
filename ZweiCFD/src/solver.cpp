#include "ZweiFoil/solver.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace zweifoil {

Solver::Solver(const Airfoil& airfoil)
    : targetAirfoil(airfoil), inviscidEngine(std::make_unique<InviscidLVPM>(airfoil)) {}

Coefficients Solver::runSimulation(const Flowconditions& conditions) {
    std::cout << "Running CFD solver at alpha = " << conditions.alpha << " degrees...\n";
    Coefficients coeffs = inviscidEngine->solve(conditions);
    coeffs.cd = calculateViscousDrag(conditions);
    return coeffs;
}

double Solver::calculateViscousDrag(const Flowconditions& conditions) {
    const auto& panels = targetAirfoil.getPanels();
    const auto& gamma = inviscidEngine->getGammaDistribution();
    
    if (panels.empty() || gamma.size() == 0) return 0.0;

    double cd_viscous = 0.0;
    double nu = conditions.kinematic_viscosity;

    for (size_t i = 0; i < panels.size(); ++i) {
        double U_e = std::abs(gamma(i));
        if (U_e < 1e-6) continue;
        
        double Re_x = (U_e * panels[i].length) / nu;
        double cf = 0.664 / std::sqrt(std::max(Re_x, 1e-6));
        
        cd_viscous += cf * panels[i].length * std::pow(U_e / conditions.V_inf, 2);
    }

    return cd_viscous;
}

const Eigen::VectorXd& Solver::getGammaDistribution() const {
    return inviscidEngine->getGammaDistribution();
}

Point2D Solver::getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const {
    return inviscidEngine->getVelocityAt(pos, conditions);
}

bool Solver::isInsideAirfoil(const Point2D& pos) const {
    return inviscidEngine->isInsideAirfoil(pos);
}

} 