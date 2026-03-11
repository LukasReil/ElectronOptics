#include "potentialMesh.hpp"

#include <tetgen.h>
#include <spdlog/spdlog.h>

using namespace ElectronOptics::Simulation::Data;

double Tetrahedron::getVolume() const {
    return glm::dot(
        glm::cross(m_vertices[1].vertex.position - m_vertices[0].vertex.position, m_vertices[2].vertex.position - m_vertices[0].vertex.position), 
        m_vertices[3].vertex.position - m_vertices[0].vertex.position) 
        / 6.0;
}

static void addElectrodeMeshToTetgenInput(const ElectrodeMesh& electrodeMesh, tetgenio& in, size_t& vertexIndexOffset, size_t& faceIndexOffset, int index) {
    for (size_t i = 0; i < electrodeMesh.getVertexCount(); i++) {
        auto vertex = electrodeMesh.getVertices().at(i);
        in.pointlist[(vertexIndexOffset + i) * 3] = vertex.x;
        in.pointlist[(vertexIndexOffset + i) * 3 + 1] = vertex.y;
        in.pointlist[(vertexIndexOffset + i) * 3 + 2] = vertex.z;
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
    size_t vertexIndex = 0;
    size_t faceIndex = 0;
    addElectrodeMeshToTetgenInput(boundingBoxMesh, in, vertexIndex, faceIndex, 1);
    for (size_t electrodeIndex = 0; electrodeIndex < electrodeMeshes.size(); electrodeIndex++) {
        addElectrodeMeshToTetgenInput(electrodeMeshes[electrodeIndex], in, vertexIndex, faceIndex, static_cast<int>(electrodeIndex) + 2);
    }

    char switches[] = "pq1.414a0.001";
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
        }
    }
}
