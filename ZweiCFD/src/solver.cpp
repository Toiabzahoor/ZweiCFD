#include "ZweiFoil/solver.hpp"  
#include <iostream> 

namespace zweifoil {  

Solver::Solver(const Airfoil& airfoil)      
     : targetAirfoil(airfoil), inviscidEngine(std::make_unique<InviscidLVPM>(airfoil)) {}  

Coefficients Solver::runInviscid(const Flowconditions& conditions) {          
     std::cout << "Running LVPM inviscid solver at alpha = " << conditions.alpha << " degrees...\n";          
     return inviscidEngine->solve(conditions); 
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

} // namespace zweifoil