#include "femSolver.hpp"

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <spdlog/spdlog.h>
#include "simulation/data/linear3d.hpp"

using namespace ElectronOptics::Simulation::Solver;
using Linear3d = ElectronOptics::Simulation::Data::Linear3d;



std::array<Linear3d, 4> calculateTentsForTetrahedron(const ElectronOptics::Simulation::Data::Tetrahedron& tetrahedron);

FEMSolver::FEMSolver(Data::PotentialMesh &inputMesh)
    : m_inputMesh(inputMesh) {
}

void FEMSolver::solve() {

    spdlog::info("Starting to solve the system of equations for the potential mesh with {} vertices and {} tetrahedra", m_inputMesh.getVertexCount(), m_inputMesh.getTetrahedra().size());

    Eigen::SparseMatrix<double> globalMatrix;
    Eigen::VectorXd solvedPotentialVector;
    size_t meshVertexCount = m_inputMesh.getVertexCount();
    Eigen::VectorXd conditionVector = Eigen::VectorXd::Zero(meshVertexCount);
    globalMatrix.resize(meshVertexCount, meshVertexCount);

    std::vector<Eigen::Triplet<double>> systemRelations;
    systemRelations.reserve(meshVertexCount * 16);


    auto& tetrahedra = m_inputMesh.getTetrahedraMutable();

    for (size_t tetrahedronIndex = 0; tetrahedronIndex < tetrahedra.size(); tetrahedronIndex++) {
        auto& tetrahedron = tetrahedra.at(tetrahedronIndex);
        auto tentFunctions = calculateTentsForTetrahedron(tetrahedron);
        tetrahedron.setTentFunctions(tentFunctions);

        double tetrahedronVolume = tetrahedron.getVolume();

        for(size_t i = 0; i < 4; i++) {
            if (tetrahedron[i].potentialIsFixed) {
                continue;
            }
            for(size_t j = 0; j < 4; j++) {
                // Could be optimized by matrix multiplication
                double integralValue = glm::dot(tentFunctions[i].gradient, tentFunctions[j].gradient) * tetrahedronVolume;
                systemRelations.emplace_back(tetrahedron[i].index, tetrahedron[j].index, integralValue);
            }
        }
    }

    auto vertices = m_inputMesh.getVertices();
    for (size_t vertexIndex = 0; vertexIndex < vertices.size(); vertexIndex++) {
        auto& vertex = vertices.at(vertexIndex);
        if (vertex.potentialIsFixed) {
            systemRelations.emplace_back(vertex.index, vertex.index, 1.0);
            conditionVector[vertex.index] = vertex.potential;
        } else {
            conditionVector[vertex.index] = 0.0;
        }
    }

    
    globalMatrix.setFromTriplets(systemRelations.begin(), systemRelations.end());
    
    spdlog::info("Finished assembling system relations: total of {} non-zero entries in the global matrix", systemRelations.size());




    Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
    solver.compute(globalMatrix);
    spdlog::info("Finished decomposing the global matrix, starting to solve the system of equations...");
    auto solveResult = solver.solve(conditionVector);
    solvedPotentialVector = solveResult;
    spdlog::info("Finished solving the system of equations, applying potentials to vertices...");
    spdlog::info("Solver info: {}", solver.info() == Eigen::Success ? "Success" : "Failure");
    spdlog::info("Solver error: {}", solver.error());


    m_inputMesh.applyPotentialsToVertices(std::vector<double>(solvedPotentialVector.begin(), solvedPotentialVector.end()));

    m_inputMesh.fixElectricField();
}



std::array<Linear3d, 4> calculateTentsForTetrahedron(const ElectronOptics::Simulation::Data::Tetrahedron& tetrahedron) {

    Eigen::Matrix4d tetrahedronMatrix({
        {1, tetrahedron[0].position.x, tetrahedron[0].position.y, tetrahedron[0].position.z},
        {1, tetrahedron[1].position.x, tetrahedron[1].position.y, tetrahedron[1].position.z},
        {1, tetrahedron[2].position.x, tetrahedron[2].position.y, tetrahedron[2].position.z},
        {1, tetrahedron[3].position.x, tetrahedron[3].position.y, tetrahedron[3].position.z}
    });
    
    // The above matrix should always be invertible, otherwise all points lie on one line
    auto decomposed = tetrahedronMatrix.partialPivLu();


    std::array<Linear3d, 4> tentFunctions;

    for(size_t i = 0; i < 4; i++) {
        Eigen::Vector4d tentProperties{
            {0.0, 0.0, 0.0, 0.0}
        };
        tentProperties[i] = 1;
        Eigen::Vector4d solveResult = decomposed.solve(tentProperties);
        tentFunctions.at(i) = Linear3d(solveResult[0], solveResult[1], solveResult[2], solveResult[3]);
    }

    return tentFunctions;
}