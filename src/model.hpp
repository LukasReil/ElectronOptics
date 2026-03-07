#pragma once

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <string>

namespace ElectronOptics {

    class Model {
    public:
        Model() = default;
        Model(const std::string& path);
        void bind() const;
        const std::vector<glm::vec3>& getVertices() const { return m_vertices; }
        GLsizei getVertexCount() const { return static_cast<GLsizei>(m_vertexCount); }

        bool operator==(const Model& other) const {
            return m_VAO == other.m_VAO; // Simple pointer comparison, assumes unique VAOs for different models
        }
    private:
        std::vector<glm::vec3> m_vertices;
        size_t m_vertexCount;
        GLuint m_VAO, m_posVBO, m_normalVBO;
    };

} // namespace ElectronOptics