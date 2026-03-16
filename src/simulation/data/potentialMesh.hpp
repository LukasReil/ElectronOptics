#pragma once

#include <vector>
#include <array>
#include "vec3d.hpp"
#include "electrodeMesh.hpp"
#include "linear3d.hpp"

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
        Tetrahedron(const std::array<VertexRef, 4>& vertices);
            
        const Vertex& operator[](size_t i) const {
            return m_vertices[i].vertex;
        }
        
        double getVolume() const;

        std::array<Linear3d, 4> getTentFunctions() const {
            return m_tentFunctions;
        }
        
        double getPotentialAtPosition(const vec3d& position) const;
        vec3d getElectricFieldAtPosition(const vec3d& position) const;
        bool isPointInside(const vec3d& position) const;
        void calculateElectricField();
    private:
        std::array<VertexRef, 4> m_vertices;
        std::array<Linear3d, 4> m_tentFunctions;
        vec3d m_electricField;
    };

    class PotentialMesh {
    public:
        PotentialMesh(const std::vector<ElectrodeMesh>& electrodeMeshes, const ElectrodeMesh& boundingBoxMesh);
        size_t getVertexCount() const {
            return m_vertices.size();
        }
        std::vector<Tetrahedron>& getTetrahedraMutable() {
            return m_tetrahedra;
        }
        const std::vector<Tetrahedron>& getTetrahedra() const {
            return m_tetrahedra;
        }
        const std::vector<Vertex>& getVertices() const {
            return m_vertices;
        }

        void applyPotentialsToVertices(const std::vector<double>& potentials);
        void fixElectricField();

        void saveAsMesh(const std::string& filename) const;
        static PotentialMesh loadFromMesh(const std::string& filename);
    private:
        PotentialMesh(std::vector<Vertex>&& vertices, std::vector<Tetrahedron>&& tetrahedra)
            : m_vertices(std::move(vertices)), m_tetrahedra(std::move(tetrahedra)) {}

        std::vector<Vertex> m_vertices;
        std::vector<Tetrahedron> m_tetrahedra;
    };

} // namespace ElectronOptics::Simulation::Data