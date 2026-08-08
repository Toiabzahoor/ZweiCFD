#pragma once

namespace zweicfd {

struct Flowconditions {
    double alpha; 
    double V_inf; 
    double kinematic_viscosity = 1.5e-5; 
    int windDirection = 0; 
};

} 
