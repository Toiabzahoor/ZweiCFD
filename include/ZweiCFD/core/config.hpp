#pragma once

#include <string>

namespace zweicfd {

struct Config {
    int targetParticleCount = 16000;
    int lbmGridNX = 128;
    int lbmGridNY = 64;
    int lbmGridNZ = 16;
    double alpha = 5.0;
    double v_inf = 1.0;
    double kinematicViscosity = 1.5e-5;
};

class ConfigLoader {
public:
    static Config load(const std::string& path);
};

} 
