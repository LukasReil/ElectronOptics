#pragma once

#include <memory>
#include <optional>

#include "potentialMesh.hpp"

namespace ElectronOptics::Simulation::Data {
    struct AABB {
        vec3d minCorner;
        vec3d maxCorner;

        bool contains(const vec3d& point) const {
            return (point.x >= minCorner.x && point.x <= maxCorner.x) &&
                (point.y >= minCorner.y && point.y <= maxCorner.y) &&
                (point.z >= minCorner.z && point.z <= maxCorner.z);
        }
    };

    class PotentialMeshOcttree {
    public:
        static PotentialMeshOcttree fromPotentialMesh(const PotentialMesh& potentialMesh, size_t maxDepth = 5);
        std::optional<Tetrahedron> getTetrahedronAtPosition(const vec3d& position) const;
    private:
        PotentialMeshOcttree(const PotentialMesh& potentialMesh)
            : m_potentialMesh(potentialMesh) {}
        const PotentialMesh& m_potentialMesh;

        struct OcttreeNode {
            vec3d minCorner;
            vec3d maxCorner;
            std::vector<size_t> tetrahedraIndices;
            std::array<std::unique_ptr<OcttreeNode>, 8> children;
            bool leaf;
        };



        std::unique_ptr<OcttreeNode> m_root;

        static std::unique_ptr<OcttreeNode> buildOcttree(const std::vector<AABB>& tetrahedronAABBs, const std::vector<size_t>& relevantTetrahedraIndices,const vec3d& minCorner, const vec3d& maxCorner, size_t maxDepth);
    };
} // namespace ElectronOptics::Simulation::Data