#pragma once

#include "ZweiCFD/solver/airfoil.hpp"
#include "ZweiCFD/solver/inviscid_lvpm.hpp"
#include <memory>

#include "ZweiCFD/solver/flowconditions.hpp"

namespace zweicfd {

struct Coefficients {
    double cl; 
    double cd; 
    double cm; 
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
