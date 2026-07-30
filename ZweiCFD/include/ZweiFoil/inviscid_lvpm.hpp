#pragma once 
#include "ZweiFoil/airfoil.hpp" 
#include <Eigen/Dense> 

namespace zweifoil { 

struct Flowconditions; 
struct Coefficients; 

class InviscidLVPM { 
public: 
    InviscidLVPM(const Airfoil& airfoil); 
         
    Coefficients solve(const Flowconditions& conditions); 
    const Eigen::VectorXd& getGammaDistribution() const { return gamma; } 

    Point2D getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const;
    
    bool isInsideAirfoil(const Point2D& pos) const;

private: 
    const Airfoil& targetAirfoil; 
    Eigen::VectorXd gamma; 
    void calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions); 
};

} 