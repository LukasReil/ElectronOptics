#include "model.hpp"

#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

using namespace ElectronOptics;

ElectronOptics::Model::Model(const std::string &path) {

    std::ifstream file(path);

    std::string modelCode;

    if (!file.is_open()) {
        spdlog::error("Failed to open model file: {}", path);
        return;
    } else {
        std::stringstream sstr;
        sstr << file.rdbuf();
        modelCode = sstr.str();
        file.close();
    }

    std::istringstream iss(modelCode);
    std::string line;

    std::vector<glm::vec3> verticesRaw;
    std::vector<glm::vec3> normalsRaw;

    std::vector<glm::vec3> normals;

    while (std::getline(iss, line)) {
        std::istringstream lineStream(line);
        std::string prefix;
        lineStream >> prefix;
        if (prefix == "v") {
            float x, y, z;
            lineStream >> x >> y >> z;
            verticesRaw.emplace_back(x, y, z);
        } else if (prefix == "vn") {
            float x, y, z;
            lineStream >> x >> y >> z;
            normalsRaw.emplace_back(x, y, z);
        } else if (prefix == "f") {
            unsigned int i1, i2, i3, n1, n2, n3;
            char slash;
            lineStream >> i1 >> slash >> slash >> n1 >> i2 >> slash >> slash >> n2 >> i3 >> slash >> slash >> n3;
            m_vertices.push_back(verticesRaw[i1 - 1]);
            m_vertices.push_back(verticesRaw[i2 - 1]);
            m_vertices.push_back(verticesRaw[i3 - 1]);
            normals.push_back(normalsRaw[n1 - 1]);
            normals.push_back(normalsRaw[n2 - 1]);
            normals.push_back(normalsRaw[n3 - 1]);
        }
    }


    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_posVBO);
    glGenBuffers(1, &m_normalVBO);
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_posVBO);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(glm::vec3), m_vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glBindBuffer(GL_ARRAY_BUFFER, m_normalVBO);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glBindVertexArray(0);
    m_vertexCount = m_vertices.size();
}

void ElectronOptics::Model::bind() const {
    glBindVertexArray(m_VAO);
}
