#include "potentialMeshOcttree.hpp"

#include <stdexcept>

using namespace ElectronOptics::Simulation::Data;

std::vector<AABB> calculateTetrahedronAABBs(const std::vector<Tetrahedron>& tetrahedra);
bool doAABBsIntersect(const AABB& a, const AABB& b);

PotentialMeshOcttree PotentialMeshOcttree::fromPotentialMesh(const PotentialMesh &potentialMesh, size_t maxDepth) {

    vec3d minCorner(std::numeric_limits<double>::max());
    vec3d maxCorner(std::numeric_limits<double>::lowest());
    for (const auto& vertex : potentialMesh.getVertices()) {
        minCorner = glm::min(minCorner, vertex.position);
        maxCorner = glm::max(maxCorner, vertex.position);
    }

    PotentialMeshOcttree octtree(potentialMesh);
    std::vector<size_t> allTetrahedraIndices(potentialMesh.getTetrahedra().size());
    for (size_t i = 0; i < potentialMesh.getTetrahedra().size(); i++) {
        allTetrahedraIndices[i] = i;
    }
    auto aabbs = calculateTetrahedronAABBs(potentialMesh.getTetrahedra());
    octtree.m_root = buildOcttree(aabbs, allTetrahedraIndices, minCorner, maxCorner, maxDepth);

    return octtree;
}

std::optional<Tetrahedron> ElectronOptics::Simulation::Data::PotentialMeshOcttree::getTetrahedronAtPosition(const vec3d &position) const {
    const OcttreeNode* currentNode = m_root.get();
    while (currentNode) {
        if (currentNode->leaf) {
            for (const auto& tetrahedronIndex : currentNode->tetrahedraIndices) {
                const auto& tetrahedron = m_potentialMesh.getTetrahedra().at(tetrahedronIndex);
                if (tetrahedron.isPointInside(position)) {
                    return tetrahedron;
                }
            }
            return std::nullopt;
        }

        size_t childIndex = 0;
        vec3d center = (currentNode->minCorner + currentNode->maxCorner) * 0.5;
        if (position.x >= center.x) childIndex |= 1;
        if (position.y >= center.y) childIndex |= 2;
        if (position.z >= center.z) childIndex |= 4;

        currentNode = currentNode->children[childIndex].get();
    }
    return std::nullopt;
}

std::unique_ptr<PotentialMeshOcttree::OcttreeNode> PotentialMeshOcttree::buildOcttree(const std::vector<AABB>& tetrahedronAABBs, const std::vector<size_t>& relevantTetrahedraIndices,const vec3d& minCorner, const vec3d& maxCorner, size_t maxDepth) {

    if (maxDepth == 0 || relevantTetrahedraIndices.size() <= 10) {
        auto node = std::make_unique<OcttreeNode>();
        node->minCorner = minCorner;
        node->maxCorner = maxCorner;
        node->tetrahedraIndices = relevantTetrahedraIndices;
        node->leaf = true;
        return node;
    }

    std::array<std::vector<size_t>, 8> childTetrahedraIndices;
    for (size_t i = 0; i < childTetrahedraIndices.size(); i++) {
        childTetrahedraIndices[i].reserve(relevantTetrahedraIndices.size() / 8 * 2);
    }
    std::array<AABB, 8> childAABBs;
    
    vec3d center = (minCorner + maxCorner) * 0.5;

    for (size_t childIndex = 0; childIndex < 8; childIndex++) {
        vec3d childMinCorner = minCorner;
        vec3d childMaxCorner = maxCorner;
        if (childIndex & 1) childMinCorner.x = center.x; else childMaxCorner.x = center.x;
        if (childIndex & 2) childMinCorner.y = center.y; else childMaxCorner.y = center.y;
        if (childIndex & 4) childMinCorner.z = center.z; else childMaxCorner.z = center.z;

        AABB childAABB{childMinCorner, childMaxCorner};
        childAABBs[childIndex] = childAABB;


        for (size_t i = 0; i < relevantTetrahedraIndices.size(); i++) {
            if (doAABBsIntersect(tetrahedronAABBs[relevantTetrahedraIndices[i]], childAABB)) {
                childTetrahedraIndices[childIndex].push_back(relevantTetrahedraIndices[i]);
            }
        }
    }

    auto node = std::make_unique<OcttreeNode>();
    node->minCorner = minCorner;
    node->maxCorner = maxCorner;
    node->leaf = false;
    for (size_t childIndex = 0; childIndex < 8; childIndex++) {
        node->children[childIndex] = buildOcttree(tetrahedronAABBs, childTetrahedraIndices[childIndex], 
            childAABBs[childIndex].minCorner, childAABBs[childIndex].maxCorner,
            maxDepth - 1);
    }



    return node;
}

std::vector<AABB> calculateTetrahedronAABBs(const std::vector<Tetrahedron>& tetrahedra) {
    std::vector<AABB> aabbs;
    aabbs.reserve(tetrahedra.size());
    for (const auto& tetrahedron : tetrahedra) {
        vec3d minCorner(std::numeric_limits<double>::max());
        vec3d maxCorner(std::numeric_limits<double>::lowest());
        for (size_t i = 0; i < 4; i++) {
            const auto& vertex = tetrahedron[i];
            minCorner = glm::min(minCorner, vertex.position);
            maxCorner = glm::max(maxCorner, vertex.position);
        }
        aabbs.push_back({minCorner, maxCorner});
    }
    return aabbs;
}

bool doAABBsIntersect(const AABB& a, const AABB& b) {
    return (a.minCorner.x <= b.maxCorner.x && a.maxCorner.x >= b.minCorner.x) &&
           (a.minCorner.y <= b.maxCorner.y && a.maxCorner.y >= b.minCorner.y) &&
           (a.minCorner.z <= b.maxCorner.z && a.maxCorner.z >= b.minCorner.z);
}