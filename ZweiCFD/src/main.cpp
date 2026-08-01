#include "simulation.hpp"
#include <memory>

int main(int argc, char* argv[]) {
    auto sim = std::make_unique<Simulation>(argc, argv);
    sim->run();
    return 0;
}