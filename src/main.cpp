
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <spdlog/spdlog.h>

#include "simulation/data/potentialMesh.hpp"
#include "simulation/data/electrodeMesh.hpp"
#include "simulation/solver/femSolver.hpp"
#include "simulation/data/potentialMeshOcttree.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


int main() {

    auto boundingBoxMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/boundary.obj", 0, 0.25, {0, 0, 0});
    auto leftElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/leftElectrode.obj", 1000, 0.02, {-0.5, 0, 0});
    auto rightElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/rightElectrode.obj", -1000, 0.02, {0.5, 0, 0});

    spdlog::info("Loaded meshes: bounding box with {} vertices and {} triangles, left electrode with {} vertices and {} triangles, right electrode with {} vertices and {} triangles",
        boundingBoxMesh.getVertexCount(), boundingBoxMesh.getTriangleCount(),
        leftElectrodeMesh.getVertexCount(), leftElectrodeMesh.getTriangleCount(),
        rightElectrodeMesh.getVertexCount(), rightElectrodeMesh.getTriangleCount());

    std::vector<ElectronOptics::Simulation::Data::ElectrodeMesh> electrodeMeshes = {leftElectrodeMesh, rightElectrodeMesh};

    ElectronOptics::Simulation::Data::PotentialMesh potentialMesh(electrodeMeshes, boundingBoxMesh);

    spdlog::info("Finished tetrahedralization: potential mesh has {} vertices and {} tetrahedra", potentialMesh.getVertexCount(), potentialMesh.getTetrahedra().size());

    ElectronOptics::Simulation::Solver::FEMSolver femSolver(potentialMesh);
    femSolver.solve();

    spdlog::info("Finished solving for potentials");

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

    return 0;
}