#pragma once

#include <vector>
#include <string>
#include <array>

#include "vec3d.hpp"

namespace ElectronOptics::Simulation::Data {

    class ElectrodeMesh {
    public:
        ElectrodeMesh(const std::vector<vec3d>& vertices, const std::vector<std::array<size_t, 3>>& triangles, double potential, double meshSize, const vec3d& pointInside) 
            : m_vertices(vertices), m_triangles(triangles), m_potential(potential), m_meshSize(meshSize), m_pointInside(pointInside) {}

        static ElectrodeMesh loadFromObjFile(const std::string& filePath, double potential, double meshSize, const vec3d& pointInside);

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

        double getMeshSize() const {
            return m_meshSize;
        }

        const vec3d& getPointInside() const {
            return m_pointInside;
        }

    private:
        std::vector<vec3d> m_vertices;
        std::vector<std::array<size_t, 3>> m_triangles;
        double m_potential;
        double m_meshSize;
        vec3d m_pointInside;
    };

} // namespace ElectronOptics::Simulation::Data