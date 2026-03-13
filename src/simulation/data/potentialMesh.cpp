#include "potentialMesh.hpp"

#include <tetgen.h>
#include <spdlog/spdlog.h>

using namespace ElectronOptics::Simulation::Data;

double getVolumeOfTetrahedronFromVertices(const vec3d& v0, const vec3d& v1, const vec3d& v2, const vec3d& v3) {
    return std::abs(glm::dot(
        glm::cross(v1 - v0, v2 - v0), 
        v3 - v0))
        / 6.0;
}

double Tetrahedron::getVolume() const {
    return getVolumeOfTetrahedronFromVertices(
        m_vertices[0].vertex.position,
        m_vertices[1].vertex.position,
        m_vertices[2].vertex.position,
        m_vertices[3].vertex.position
    );
}

double ElectronOptics::Simulation::Data::Tetrahedron::getPotentialAtPosition(const vec3d &position) const {
    double potential = 0.0;
    for (size_t i = 0; i < 4; i++) {
        potential += m_tentFunctions[i].eval(position) * m_vertices[i].vertex.potential;
    }
    return potential;
}

vec3d ElectronOptics::Simulation::Data::Tetrahedron::getElectricFieldAtPosition(const vec3d &position) const {
    vec3d electricField(0.0, 0.0, 0.0);
    for (size_t i = 0; i < 4; i++) {
        electricField += m_tentFunctions[i].gradient * m_vertices[i].vertex.potential;
    }
    return electricField;
}

bool ElectronOptics::Simulation::Data::Tetrahedron::isPointInside(const vec3d & position) const {
    double totalVolume = getVolume();
    double volume1 = getVolumeOfTetrahedronFromVertices(
        m_vertices[0].vertex.position,
        m_vertices[1].vertex.position,
        m_vertices[2].vertex.position,
        position
    );
    double volume2 = getVolumeOfTetrahedronFromVertices(
        m_vertices[0].vertex.position,
        m_vertices[1].vertex.position,
        m_vertices[3].vertex.position,
        position
    );
    double volume3 = getVolumeOfTetrahedronFromVertices(
        m_vertices[0].vertex.position,
        m_vertices[2].vertex.position,
        m_vertices[3].vertex.position,
        position
    );
    double volume4 = getVolumeOfTetrahedronFromVertices(
        m_vertices[1].vertex.position,
        m_vertices[2].vertex.position,
        m_vertices[3].vertex.position,
        position
    );

    double volumeSum = volume1 + volume2 + volume3 + volume4;
    double volumeDifference = std::abs(totalVolume - volumeSum);
    return volumeDifference / totalVolume < 1e-10;
}

void ElectronOptics::Simulation::Data::Tetrahedron::calculateElectricField() {
    m_electricField = vec3d(0.0, 0.0, 0.0);
    for (size_t i = 0; i < 4; i++) {
        m_electricField += m_tentFunctions[i].gradient * m_vertices[i].vertex.potential;
    }
}

static void addElectrodeMeshToTetgenInput(const ElectrodeMesh& electrodeMesh, tetgenio& in, size_t& vertexIndexOffset, size_t& faceIndexOffset, int index) {
    for (size_t i = 0; i < electrodeMesh.getVertexCount(); i++) {
        auto vertex = electrodeMesh.getVertices().at(i);
        in.pointlist[(vertexIndexOffset + i) * 3] = vertex.x;
        in.pointlist[(vertexIndexOffset + i) * 3 + 1] = vertex.y;
        in.pointlist[(vertexIndexOffset + i) * 3 + 2] = vertex.z;
        in.pointmtrlist[vertexIndexOffset + i] = electrodeMesh.getMeshSize();
    }

    for (size_t i = 0; i < electrodeMesh.getTriangleCount(); i++) {
        auto triangle = electrodeMesh.getTriangles().at(i);
        auto& facet = in.facetlist[faceIndexOffset + i];
        facet.numberofpolygons = 1;
        facet.holelist = nullptr;
        facet.polygonlist = new tetgenio::polygon[1];
        facet.polygonlist[0].numberofvertices = 3;
        facet.polygonlist[0].vertexlist = new int[3];
        facet.polygonlist[0].vertexlist[0] = triangle[0] + vertexIndexOffset;
        facet.polygonlist[0].vertexlist[1] = triangle[1] + vertexIndexOffset;
        facet.polygonlist[0].vertexlist[2] = triangle[2] + vertexIndexOffset;
        in.facetmarkerlist[faceIndexOffset + i] = index;
    }

    vertexIndexOffset += electrodeMesh.getVertexCount();
    faceIndexOffset += electrodeMesh.getTriangleCount();
}

PotentialMesh::PotentialMesh(const std::vector<ElectrodeMesh> &electrodeMeshes, const ElectrodeMesh &boundingBoxMesh) {

    tetgenio in, out;

    in.firstnumber = 0;
    in.mesh_dim = 3;

    size_t vertexCount = boundingBoxMesh.getVertexCount();
    for (const auto& electrodeMesh : electrodeMeshes) {
        vertexCount += electrodeMesh.getVertexCount();
    }

    size_t triangleCount = boundingBoxMesh.getTriangleCount();
    for (const auto& electrodeMesh : electrodeMeshes) {
        triangleCount += electrodeMesh.getTriangleCount();
    }

    in.numberofpoints = vertexCount;
    in.pointlist = new REAL[vertexCount * 3];
    in.numberoffacets = triangleCount;
    in.facetlist = new tetgenio::facet[triangleCount];
    in.facetmarkerlist = new int[triangleCount];
    in.numberofpointmtrs = 1;
    in.pointmtrlist = new REAL[vertexCount * in.numberofpointmtrs];
    in.numberofholes = electrodeMeshes.size();
    in.holelist = new REAL[in.numberofholes * 3];
    size_t vertexIndex = 0;
    size_t faceIndex = 0;
    addElectrodeMeshToTetgenInput(boundingBoxMesh, in, vertexIndex, faceIndex, 1);
    for (size_t electrodeIndex = 0; electrodeIndex < electrodeMeshes.size(); electrodeIndex++) {
        addElectrodeMeshToTetgenInput(electrodeMeshes[electrodeIndex], in, vertexIndex, faceIndex, static_cast<int>(electrodeIndex) + 2);
    }
    for (size_t i = 0; i < electrodeMeshes.size(); i++) {
        in.holelist[i * 3 + 0] = electrodeMeshes[i].getPointInside().x;
        in.holelist[i * 3 + 1] = electrodeMeshes[i].getPointInside().y;
        in.holelist[i * 3 + 2] = electrodeMeshes[i].getPointInside().z;
    }

    char switches[] = "pq1.2m";
    spdlog::info("Starting tetrahedralization...");
    tetrahedralize(switches, &in, &out);
    spdlog::info("Finished tetrahedralization: generated {} vertices and {} tetrahedra", out.numberofpoints, out.numberoftetrahedra);

    m_vertices.reserve(out.numberofpoints);
    for (size_t i = 0; i < out.numberofpoints; i++) {
        Vertex vertex;
        vertex.position = vec3d(out.pointlist[i * 3], out.pointlist[i * 3 + 1], out.pointlist[i * 3 + 2]);
        vertex.index = i;
        vertex.potential = 0.0;
        vertex.potentialIsFixed = false;
        m_vertices.push_back(vertex);
    }

    spdlog::info("Finished generating vertices: total of {} vertices", m_vertices.size());

    m_tetrahedra.reserve(out.numberoftetrahedra);
    for (size_t i = 0; i < out.numberoftetrahedra; i++) {
        std::array<VertexRef, 4> vertices = {
            VertexRef{m_vertices[out.tetrahedronlist[i * 4]]},
            VertexRef{m_vertices[out.tetrahedronlist[i * 4 + 1]]},
            VertexRef{m_vertices[out.tetrahedronlist[i * 4 + 2]]},
            VertexRef{m_vertices[out.tetrahedronlist[i * 4 + 3]]}
        };
        m_tetrahedra.emplace_back(vertices);
    }

    spdlog::info("Finished generating tetrahedra: total of {} tetrahedra", m_tetrahedra.size());
    for (size_t i = 0; i < out.numberoftrifaces; i++) {
        int marker = out.trifacemarkerlist[i];
        if (marker > 1) {
            double potential = electrodeMeshes[marker - 2].getPotential();
            m_vertices[out.trifacelist[i * 3]].potential = potential;
            m_vertices[out.trifacelist[i * 3]].potentialIsFixed = true;
            m_vertices[out.trifacelist[i * 3 + 1]].potential = potential;
            m_vertices[out.trifacelist[i * 3 + 1]].potentialIsFixed = true;
            m_vertices[out.trifacelist[i * 3 + 2]].potential = potential;
            m_vertices[out.trifacelist[i * 3 + 2]].potentialIsFixed = true;
        }
    }
    spdlog::info("Finished applying boundary conditions to vertices");

}

void ElectronOptics::Simulation::Data::PotentialMesh::applyPotentialsToVertices(const std::vector<double> &potentials) {
    for (size_t vertexIndex = 0; vertexIndex < m_vertices.size(); vertexIndex++) {
        if (!m_vertices[vertexIndex].potentialIsFixed) {
            m_vertices[vertexIndex].potential = potentials[vertexIndex];
        } else {
            // Sanity check: fixed potential should not be changed
            if (std::abs(m_vertices[vertexIndex].potential - potentials[vertexIndex]) > 1e-6) {
                spdlog::warn("Potential at vertex {} is fixed at {}, but solver returned a different value of {}", vertexIndex, m_vertices[vertexIndex].potential, potentials[vertexIndex]);
            }
        }
    }
}

void ElectronOptics::Simulation::Data::PotentialMesh::fixElectricField() {

    for (auto& tetrahedron : m_tetrahedra) {
        tetrahedron.calculateElectricField();
    }

}
