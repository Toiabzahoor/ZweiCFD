#pragma once
#include "ZweiFoil/airfoil.hpp"

namespace zweifoil {

struct Flowconditions {
    double alpha; //AOA in deg
    double V_inf; //freestream velocity
};

struct Coefficients{
    double cl; //lift coeff.
    double cd; //drag coeff
    double cm; //moment coeff.

};

class Solver {
public:
    Solver(const Airfoil& airfoil);

    Coefficients runInviscid(const Flowconditions& conditions);
private:
    Airfoil targetAirfoil;

};

}