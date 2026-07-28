#pragma once
#include "ZweiFoil/airfoil.hpp"
//eigen for Linear algebra
#include <Eigen/Dense>

namespace zweifoil {

// Forward declarations
struct Flowconditions;
struct Coefficients;

//calculating inviscid before we add viscid effects

class InviscidLVPM {
public:
    InviscidLVPM(const Airfoil& airfoil);
    
    Coefficients solve(const Flowconditions& conditions);
    const Eigen::VectorXd& getGammaDistribution() const { return gamma; }

private:
    const Airfoil& targetAirfoil;
    Eigen::VectorXd gamma;
    void calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions);
};

}