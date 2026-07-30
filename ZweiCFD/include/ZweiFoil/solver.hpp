#pragma once  

#include "ZweiFoil/airfoil.hpp"  
#include "ZweiFoil/inviscid_lvpm.hpp" 
#include <memory> 

namespace zweifoil {  

struct Flowconditions {          
    double alpha;          // AoA in degrees          
    double V_inf;          // freestream velocity  
    int windDirection = 0; // 0=Left, 1=Right, 2=Top, 3=Bottom
}; 

struct Coefficients {          
    double cl; // lift coeff.          
    double cd; // drag coeff          
    double cm; // moment coeff.  
}; 

class Solver {  
public:          
    Solver(const Airfoil& airfoil);          
    Coefficients runInviscid(const Flowconditions& conditions);      
    const Eigen::VectorXd& getGammaDistribution() const; 
    Point2D getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const;
    
    // New: check if a point lies inside the airfoil
    bool isInsideAirfoil(const Point2D& pos) const;

private:          
    Airfoil targetAirfoil;      
    std::unique_ptr<InviscidLVPM> inviscidEngine; 
}; 

} // namespace zweifoil