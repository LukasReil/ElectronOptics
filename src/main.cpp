
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <spdlog/spdlog.h>
#include <random>

#include "simulation/data/potentialMesh.hpp"
#include "simulation/data/electrodeMesh.hpp"
#include "simulation/solver/femSolver.hpp"
#include "simulation/data/potentialMeshOcttree.hpp"
#include "simulation/tracer/tracer.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

const vec3d electronSourcePosition(0, 7, 0);
const vec3d electronSourceDirection(0, -1, 0);
const double Q = -1.602176634e-19; // Charge of an electron in coulombs
const double M = 9.10938356e-31; // Mass of an electron in kg

const double V_Accel = 1000; // Acceleration voltage in volts
const vec3d initialVelocity = electronSourceDirection * std::sqrt(-2 * Q * V_Accel / M); // Initial velocity of the electron

const size_t NUM_ELECTRONS = 50; // Number of electrons to simulate

int main() {

    // auto boundingBoxMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/boundary.obj", 0, 0.25, {0, 0, 0});
    // auto leftElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/leftElectrode.obj", 1000, 0.02, {-0.5, 0, 0});
    // auto rightElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/rightElectrode.obj", -1000, 0.02, {0.5, 0, 0});

    auto einzelBoundaryMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/EinzelBoundary.obj", 0, 1, {0, 0, 0});
    auto einzelElectrode1Mesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/Electrode1.obj", 0, 0.1, {0.9, 2, 0});
    auto einzelElectrode2Mesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/Electrode2.obj", 1000, 0.1, {0.9, 0, 0});
    auto einzelElectrode3Mesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/Electrode3.obj", 0, 0.1, {0.9, -2, 0});

    // spdlog::info("Loaded meshes: bounding box with {} vertices and {} triangles, left electrode with {} vertices and {} triangles, right electrode with {} vertices and {} triangles",
    //     boundingBoxMesh.getVertexCount(), boundingBoxMesh.getTriangleCount(),
    //     leftElectrodeMesh.getVertexCount(), leftElectrodeMesh.getTriangleCount(),
    //     rightElectrodeMesh.getVertexCount(), rightElectrodeMesh.getTriangleCount());

    std::vector<ElectronOptics::Simulation::Data::ElectrodeMesh> electrodeMeshes = {einzelElectrode1Mesh, einzelElectrode2Mesh, einzelElectrode3Mesh};

    ElectronOptics::Simulation::Data::PotentialMesh potentialMesh(electrodeMeshes, einzelBoundaryMesh);

    spdlog::info("Finished tetrahedralization: potential mesh has {} vertices and {} tetrahedra", potentialMesh.getVertexCount(), potentialMesh.getTetrahedra().size());

    ElectronOptics::Simulation::Solver::FEMSolver femSolver(potentialMesh);
    femSolver.solve();
    spdlog::info("Finished solving for potentials");

    potentialMesh.saveAsMesh("potentialMesh.mesh");
    return 0;

    // auto potentialMesh = ElectronOptics::Simulation::Data::PotentialMesh::loadFromMesh("potentialMesh.mesh");
    spdlog::info("Finished loading potential mesh from file: potential mesh has {} vertices and {} tetrahedra", potentialMesh.getVertexCount(), potentialMesh.getTetrahedra().size());


    auto file = std::ofstream("solvedPotentials.txt");
    auto vertices = potentialMesh.getVertices();
    for (size_t vertexIndex = 0; vertexIndex < vertices.size(); vertexIndex++) {
        auto& vertex = vertices.at(vertexIndex);
        file << vertex.position.x << " " << vertex.position.y << " " << vertex.position.z << " " << vertex.potential << std::endl;
    }
    file.close();

    spdlog::info("Finished writing solved potentials to file");

    auto octtree = ElectronOptics::Simulation::Data::PotentialMeshOcttree::fromPotentialMesh(potentialMesh, 8);
    auto tetrahedron = octtree.getTetrahedronAtPosition({0, 0, 0});
    auto electricField = tetrahedron ? tetrahedron->getElectricFieldAtPosition({0, 0, 0}) : vec3d(0);
    spdlog::info("Measured E-field at (0, 0, 0): {}", glm::to_string(electricField));

    auto tracer = ElectronOptics::Simulation::Tracer::Tracer(octtree, {3e-14, 30000, Q, M});

    file = std::ofstream("electronPath.txt");

    auto rng = std::default_random_engine{};
    auto posDist = std::normal_distribution<double>{0, 0.05}; // 0.1 mm standard deviation for initial position

    for (size_t i = 0; i < NUM_ELECTRONS; i++) {

        vec3d electronPositionOffset = vec3d(posDist(rng), 0, posDist(rng));

        auto path = tracer.traceElectronPath(electronSourcePosition + electronPositionOffset, initialVelocity * 1e3); // * 1e3 for converting from m/s to mm/s, since the geometry is in mm
        spdlog::info("Traced electron path with {} steps", path.size());

        for (const auto& position : path) {
            file << position.x << " " << position.y << " " << position.z << " ";
        }
        file << std::endl;
    }
    file.close();

    spdlog::info("Finished writing electron path to file");

    return 0;
}