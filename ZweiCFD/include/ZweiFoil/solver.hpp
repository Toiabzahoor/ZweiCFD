#pragma once

#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/inviscid_lvpm.hpp"
#include <memory>

namespace zweifoil {


struct Flowconditions {
    double alpha; // AoA in deg
    double V_inf; // freestream velocity
    double kinematic_viscosity = 1.5e-5; 
    int windDirection = 0; // 0=Left, 1 = Right, 2= Top, 3= bottom

};

struct Coefficients {
    double cl; // lift coeff
    double cd; // drag coeff
    double cm; //moment coeff
};

class Solver {
public:
    Solver(const Airfoil& airfoil);
    Coefficients runSimulation(const Flowconditions& conditions);
    const Eigen::VectorXd& getGammaDistribution() const;
    Point2D getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const;
    bool isInsideAirfoil(const Point2D& pos) const;

    InviscidLVPM* getInviscidEngine() const { return inviscidEngine.get(); }

private:
    Airfoil targetAirfoil;
    std::unique_ptr<InviscidLVPM> inviscidEngine;
    double calculateViscousDrag(const Flowconditions& conditions);

};

}