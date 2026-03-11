
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <spdlog/spdlog.h>

#include "simulation/data/potentialMesh.hpp"
#include "simulation/data/electrodeMesh.hpp"
#include "simulation/solver/femSolver.hpp"



int main() {

    auto boundingBoxMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/boundary.obj", 0);
    auto leftElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/leftElectrode.obj", 1000);
    auto rightElectrodeMesh = ElectronOptics::Simulation::Data::ElectrodeMesh::loadFromObjFile("assets/rightElectrode.obj", -1000);

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

    return 0;
}