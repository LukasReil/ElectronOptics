#include "electrodeMesh.hpp"

#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>

using namespace ElectronOptics::Simulation::Data;

ElectrodeMesh ElectrodeMesh::loadFromObjFile(const std::string &filePath, double potential) {
    
    std::ifstream file(filePath);

    std::string modelCode;

    if (!file.is_open()) {
        spdlog::error("Failed to open model file: {}", filePath);
        throw std::runtime_error("Failed to open model file");
    } else {
        std::stringstream sstr;
        sstr << file.rdbuf();
        modelCode = sstr.str();
        file.close();
    }

    
    std::istringstream iss(modelCode);
    std::string line;

    std::vector<vec3d> vertices;
    std::vector<std::array<size_t, 3>> triangles;

    while (std::getline(iss, line)) {
        std::istringstream lineStream(line);
        std::string prefix;
        lineStream >> prefix;
        if (prefix == "v") {
            float x, y, z;
            lineStream >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        } else if (prefix == "f") {
            unsigned int i1, i2, i3;
            lineStream >> i1 >> i2 >> i3;
            triangles.emplace_back(std::array<size_t, 3>{i1 - 1, i2 - 1, i3 - 1});
        }
    }
    
    return ElectrodeMesh(vertices, triangles, potential);
}