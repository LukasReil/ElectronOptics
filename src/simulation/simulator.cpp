#include "simulator.hpp"

#include <chrono>
#include <spdlog/spdlog.h>
#include <Eigen/IterativeLinearSolvers>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace ElectronOptics::Simulation;

Simulator::Simulator(double gridResolution_mm, glm::vec3 gridMin, glm::vec3 gridMax, std::function<bool(glm::vec3)> isInsideGeometry, std::function<double(glm::vec3)> potentialFunction) 
    : m_gridResolution_mm(gridResolution_mm), m_gridMin(gridMin), m_gridMax(gridMax), 
    m_isInsideGeometry(isInsideGeometry), m_potentialFunction(potentialFunction) {

    m_gridSizeX = static_cast<size_t>((m_gridMax.x - m_gridMin.x) / m_gridResolution_mm) + 1;
    m_gridSizeY = static_cast<size_t>((m_gridMax.y - m_gridMin.y) / m_gridResolution_mm) + 1;
    m_gridSizeZ = static_cast<size_t>((m_gridMax.z - m_gridMin.z) / m_gridResolution_mm) + 1;

    m_totalGridPoints = m_gridSizeX * m_gridSizeY * m_gridSizeZ;

    spdlog::info("Grid size: {} x {} x {} ({} total points)", m_gridSizeX, m_gridSizeY, m_gridSizeZ, m_totalGridPoints);

    constructSparseMatrix();
}

void ElectronOptics::Simulation::Simulator::solvePotential() {

    Eigen::setNbThreads(6);

    // Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IncompleteLUT<double>> solver;
    Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
    solver.setMaxIterations(10000);
    solver.setTolerance(1);
    
    spdlog::info("Solving linear system with BiCGSTAB solver...");
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    solver.compute(m_sparseMatrix);
    spdlog::info("Finished computing preconditioner. Starting solve...");
    std::chrono::steady_clock::time_point afterCompute = std::chrono::steady_clock::now();
    m_potential = solver.solve(m_b);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    spdlog::info("Solver {}", solver.info() == Eigen::Success ? "converged" : "did not converge");
    spdlog::info("Solver iterations: {}", solver.iterations());
    spdlog::info("Solver error: {}", solver.error());

    spdlog::info("Time taken for compute: {} seconds", std::chrono::duration_cast<std::chrono::seconds>(afterCompute - start).count());
    spdlog::info("Time taken for solve: {} seconds", std::chrono::duration_cast<std::chrono::seconds>(end - afterCompute).count());
    spdlog::info("Time taken total: {} seconds", std::chrono::duration_cast<std::chrono::seconds>(end - start).count());
}

double ElectronOptics::Simulation::Simulator::getPotentialAtPosition(glm::vec3 position) const {

    if (position.x < m_gridMin.x || position.x > m_gridMax.x ||
        position.y < m_gridMin.y || position.y > m_gridMax.y ||
        position.z < m_gridMin.z || position.z > m_gridMax.z) {
        spdlog::warn("Position {} is out of bounds. Returning 0 potential.", glm::to_string(position));
        return 0.0;
    }

    double xIndex = (position.x - m_gridMin.x) / m_gridResolution_mm;
    double yIndex = (position.y - m_gridMin.y) / m_gridResolution_mm;
    double zIndex = (position.z - m_gridMin.z) / m_gridResolution_mm;

    size_t x0 = static_cast<size_t>(std::floor(xIndex));
    size_t y0 = static_cast<size_t>(std::floor(yIndex));
    size_t z0 = static_cast<size_t>(std::floor(zIndex));
    size_t x1 = static_cast<size_t>(std::ceil(xIndex));
    size_t y1 = static_cast<size_t>(std::ceil(yIndex));
    size_t z1 = static_cast<size_t>(std::ceil(zIndex));

    double xd = xIndex - x0;
    double yd = yIndex - y0;
    double zd = zIndex - z0;

    // Trilinear interpolation

    double c00 = m_potential[getGridIndex(x0, y0, z0)] * (1 - xd) + m_potential[getGridIndex(x1, y0, z0)] * xd;
    double c01 = m_potential[getGridIndex(x0, y0, z1)] * (1 - xd) + m_potential[getGridIndex(x1, y0, z1)] * xd;
    double c10 = m_potential[getGridIndex(x0, y1, z0)] * (1 - xd) + m_potential[getGridIndex(x1, y1, z0)] * xd;
    double c11 = m_potential[getGridIndex(x0, y1, z1)] * (1 - xd) + m_potential[getGridIndex(x1, y1, z1)] * xd;

    double c0 = c00 * (1 - yd) + c10 * yd;
    double c1 = c01 * (1 - yd) + c11 * yd;

    return c0 * (1 - zd) + c1 * zd;
}

size_t ElectronOptics::Simulation::Simulator::getGridIndex(size_t x, size_t y, size_t z) const {
    return x + m_gridSizeX * (y + m_gridSizeY * z);
}

void ElectronOptics::Simulation::Simulator::constructSparseMatrix() {

    spdlog::info("Constructing sparse matrix...");
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(m_totalGridPoints * 7); // Each point can have up to 7 non-zero entries (itself + 6 neighbors)
    m_b = Eigen::VectorXd::Zero(m_totalGridPoints);

    for(size_t ix = 0; ix < m_gridSizeX; ++ix) {
        for(size_t iy = 0; iy < m_gridSizeY; ++iy) {
            for(size_t iz = 0; iz < m_gridSizeZ; ++iz) {
                size_t index = getGridIndex(ix, iy, iz);
                glm::vec3 point = getPhysicalPosition(ix, iy, iz);

                if (isBoundary(ix, iy, iz)) {
                    tripletList.emplace_back(index, index, 1.0);
                    m_b[index] = 0; // Boundary condition: potential is zero at the boundaries
                } else if (m_isInsideGeometry(point)) {
                    tripletList.emplace_back(index, index, 1.0);
                    m_b[index] = m_potentialFunction(point);
                } else {
                    tripletList.emplace_back(index, index, -6.0);
                    tripletList.emplace_back(index, getGridIndex(ix + 1, iy, iz), 1.0);
                    tripletList.emplace_back(index, getGridIndex(ix - 1, iy, iz), 1.0);
                    tripletList.emplace_back(index, getGridIndex(ix, iy + 1, iz), 1.0);
                    tripletList.emplace_back(index, getGridIndex(ix, iy - 1, iz), 1.0);
                    tripletList.emplace_back(index, getGridIndex(ix, iy, iz + 1), 1.0);
                    tripletList.emplace_back(index, getGridIndex(ix, iy, iz - 1), 1.0);
                    m_b[index] = 0; // Source term is zero inside the geometry
                }
            }
        }
    }

    
    m_sparseMatrix.resize(m_totalGridPoints, m_totalGridPoints);
    m_sparseMatrix.setFromTriplets(tripletList.begin(), tripletList.end());
    
    spdlog::info("Sparse matrix constructed with {} non-zero entries", tripletList.size());
}

bool ElectronOptics::Simulation::Simulator::isBoundary(size_t x, size_t y, size_t z) const {
    return x == 0 || x == m_gridSizeX - 1 || y == 0 || y == m_gridSizeY - 1 || z == 0 || z == m_gridSizeZ - 1;
}

glm::vec3 ElectronOptics::Simulation::Simulator::getPhysicalPosition(size_t x, size_t y, size_t z) const {
    return m_gridMin + glm::vec3(x * m_gridResolution_mm, y * m_gridResolution_mm, z * m_gridResolution_mm);
}
