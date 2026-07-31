#pragma once

#include <string>
#include <memory>
#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/solver.hpp"
#include "raylib.h"

struct WindParticle {
    Vector2 pos;
    Vector2 prevPos;
    float baseSize;
    float alpha;
    float speedJitter;
    bool active;
    float age;
};

struct WindSystem {
    static constexpr int MAX_PARTICLES = 16000;
    WindParticle particles[MAX_PARTICLES];
    int activeCount = 0;
};

class Simulation {
public:
    Simulation(int argc, char* argv[]);
    ~Simulation();
    void run();

private:
    float randFloat(float min, float max);
    Vector2 getWindArrowAngle(int windDir);
    void spawnParticle(WindParticle& p, int windDirection, const Vector2& spawnTL, const Vector2& spawnBR);
    void rebuildSolverWithRotation();

    zweifoil::Airfoil foil;
    zweifoil::Airfoil rotatedFoil;
    zweifoil::Flowconditions flow;
    std::unique_ptr<zweifoil::Solver> solver;
    zweifoil::Coefficients results;

    Camera2D camera;
    WindSystem wind;
    
    float particleSpawnTimer;
    float displayedVelocity;
    Vector2 windArrowDir;
    int current_sim;
    char fileBuf[256];
};