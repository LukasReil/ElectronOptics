#pragma once

#include <vector>
#include <array>
#include "vec3d.hpp"
#include "electrodeMesh.hpp"

namespace ElectronOptics::Simulation::Data {

    class Vertex {
    public:
        vec3d position;
        size_t index;
        double potential;
        bool potentialIsFixed;
    };

    struct VertexRef {
        Vertex& vertex;
    };

    class Tetrahedron {
    public:
        Tetrahedron(const std::array<VertexRef, 4>& vertices)
            : m_vertices(vertices) {}
            
        const Vertex& operator[](size_t i) const {
            return m_vertices[i].vertex;
        }
        
        double getVolume() const;
    private:
        std::array<VertexRef, 4> m_vertices;
    };

    class PotentialMesh {
    public:
        PotentialMesh(const std::vector<ElectrodeMesh>& electrodeMeshes, const ElectrodeMesh& boundingBoxMesh);
        size_t getVertexCount() const {
            return m_vertices.size();
        }
        const std::vector<Tetrahedron>& getTetrahedra() const {
            return m_tetrahedra;
        }
        const std::vector<Vertex>& getVertices() const {
            return m_vertices;
        }

        void applyPotentialsToVertices(const std::vector<double>& potentials);
    private:
        std::vector<Vertex> m_vertices;
        std::vector<Tetrahedron> m_tetrahedra;
    };

} // namespace ElectronOptics::Simulation::Data