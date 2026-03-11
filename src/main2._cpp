#include "simulation/simulator.hpp"


#include <fstream>
#include <sstream>

static const glm::vec3 GRID_MIN{-50, -100, -50};
static const glm::vec3 GRID_MAX{50, 0, 50};
static const double GRID_RESOLUTION_MM = 0.25; 



int main() {

    ElectronOptics::Simulation::Simulator simulator(GRID_RESOLUTION_MM, GRID_MIN, GRID_MAX,
        [](glm::vec3 pos) {
            // Cylinder centered at (0, -50, 0) with radius 10 and height 20
            glm::vec3 center(0, -50, 0);
            float radius = 10.0f;
            float height = 20.0f;
            return glm::length(glm::vec3(pos.x - center.x, pos.y - center.y, 0)) < radius && pos.z >= center.z - height/2 && pos.z <= center.z + height/2;
        },
        [](glm::vec3 pos) {
            // Equipotential in the cyclinder: 1kV
            return 1000;
        }
    );

    simulator.solvePotential();

    const double SAMPLE_RESOLUTION_MM = 0.1;

    double xSpann = GRID_MAX.x - GRID_MIN.x;
    double ySpann = GRID_MAX.y - GRID_MIN.y;


    size_t sampleCountX = static_cast<size_t>(xSpann / SAMPLE_RESOLUTION_MM) + 1;
    size_t sampleCountY = static_cast<size_t>(ySpann / SAMPLE_RESOLUTION_MM) + 1;
    double* potentialMap = new double[sampleCountX * sampleCountY];
    for (size_t i = 0; i < sampleCountX * sampleCountY; ++i) {
        potentialMap[i] = 0.0;
    }
    // Sample 2d slice of the potential map at z=0
    double zSlice = 0;
    for (double x = 0; x < xSpann; x += SAMPLE_RESOLUTION_MM) {
        for (double y = 0; y < ySpann; y += SAMPLE_RESOLUTION_MM) {
            size_t index = static_cast<size_t>(x / SAMPLE_RESOLUTION_MM + (y / SAMPLE_RESOLUTION_MM) * sampleCountX);
            potentialMap[index] = simulator.getPotentialAtPosition(glm::vec3(GRID_MIN.x + x, GRID_MIN.y + y, zSlice));
        }
    }

    std::ofstream outFile("potential_map.txt");
    for (size_t y = 0; y < sampleCountY; ++y) {
        for (size_t x = 0; x < sampleCountX; ++x) {
            size_t index = x + y * sampleCountX;
            outFile << potentialMap[index];
            if (x < sampleCountX - 1) {
                outFile << ",";
            }
        }
        outFile << "\n";
    }

    outFile.close();
    

    return 0;
}