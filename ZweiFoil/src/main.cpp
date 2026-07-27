#include <iostream>
#include <string>
#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/solver.hpp"

int main(int argc, char* argv[]) {
    std::cout << "==== ZweiFoil v0.0 ===\n";

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <airfoil.dat>\n";
        return 1;
    }

    std::string filename = argv[1];

    //initializing
    zweifoil::Airfoil foil;
    if (!foil.loadFromFile(filename)) {
        std::cerr << "Error: Failed to load Airfoil Data!.\n";
        return 1;

    }
    std::cout << "Loaded: " << foil.getName() << "\n";

    // defining flow conditions
    zweifoil::Flowconditions flow;
    flow.alpha = 5.0; // 5 deg AoA
    flow.V_inf = 1.0;

    // running solver.
    zweifoil::Solver solver(foil);
    zweifoil::Coefficients results = solver.runInviscid(flow);
    // output
    std::cout << "\nResults:\n"
              << " Cl: " << results.cl << "\n"
              << " Cd: " << results.cd << "\n"
              << " Cm: " << results.cm << "\n";

    return 0;
}