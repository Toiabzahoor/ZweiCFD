#include "ZweiCFD/core/config.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace zweicfd {

Config ConfigLoader::load(const std::string& path) {
    Config config;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[Config] Failed to open " << path << ". Using defaults.\n";
        return config;
    }

    try {
        json j;
        file >> j;
        
        if (j.contains("targetParticleCount")) config.targetParticleCount = j["targetParticleCount"];
        if (j.contains("lbmGridNX")) config.lbmGridNX = j["lbmGridNX"];
        if (j.contains("lbmGridNY")) config.lbmGridNY = j["lbmGridNY"];
        if (j.contains("lbmGridNZ")) config.lbmGridNZ = j["lbmGridNZ"];
        if (j.contains("alpha")) config.alpha = j["alpha"];
        if (j.contains("v_inf")) config.v_inf = j["v_inf"];
        if (j.contains("kinematicViscosity")) config.kinematicViscosity = j["kinematicViscosity"];
        
        std::cout << "[Config] Successfully loaded config from " << path << "\n";
    } catch (json::exception& e) {
        std::cerr << "[Config] Error parsing JSON: " << e.what() << "\n";
    }

    return config;
}

} 
