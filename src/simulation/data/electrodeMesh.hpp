#pragma once

#include <vector>
#include <string>
#include <array>

#include "vec3d.hpp"

namespace ElectronOptics::Simulation::Data {

    class ElectrodeMesh {
    public:
        ElectrodeMesh(const std::vector<vec3d>& vertices, const std::vector<std::array<size_t, 3>>& triangles, double potential)
            : m_vertices(vertices), m_triangles(triangles), m_potential(potential) {}

        static ElectrodeMesh loadFromObjFile(const std::string& filePath, double potential);

        const std::vector<vec3d>& getVertices() const {
            return m_vertices;
        }

        size_t getVertexCount() const {
            return m_vertices.size();
        }
        
        const std::vector<std::array<size_t, 3>>& getTriangles() const {
            return m_triangles;
        }

        size_t getTriangleCount() const {
            return m_triangles.size();
        }

        double getPotential() const {
            return m_potential;
        }

    private:
        std::vector<vec3d> m_vertices;
        std::vector<std::array<size_t, 3>> m_triangles;
        double m_potential;
    };

} // namespace ElectronOptics::Simulation::Data