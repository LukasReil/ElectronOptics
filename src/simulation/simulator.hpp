#pragma once

#include <glm/glm.hpp>
#include <Eigen/Sparse>
#include <vector>

namespace ElectronOptics::Simulation {
    class Simulator {
    public:
        Simulator(double gridResolution_mm, glm::vec3 gridMin, glm::vec3 gridMax, std::function<bool (glm::vec3)> isInsideGeometry, std::function<double (glm::vec3)> potentialFunction);

        void solvePotential();
        double getPotentialAtPosition(glm::vec3 position) const;
    private:
        double m_gridResolution_mm;
        glm::vec3 m_gridMin;
        glm::vec3 m_gridMax;
        std::function<bool (glm::vec3)> m_isInsideGeometry;
        std::function<double (glm::vec3)> m_potentialFunction;
        size_t m_gridSizeX;
        size_t m_gridSizeY;
        size_t m_gridSizeZ;
        size_t m_totalGridPoints;

        Eigen::SparseMatrix<double> m_sparseMatrix;
        Eigen::VectorXd m_b;
        Eigen::VectorXd m_potential;

        size_t getGridIndex(size_t x, size_t y, size_t z) const;
        void constructSparseMatrix();
        bool isBoundary(size_t x, size_t y, size_t z) const;
        glm::vec3 getPhysicalPosition(size_t x, size_t y, size_t z) const;
    };
} // namespace ElectronOptics::Simulation