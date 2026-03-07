#pragma once

#include <glm/glm.hpp>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "model.hpp"

namespace ElectronOptics {

    class ModelRenderInfo {

        public:
            const Model& model;
            glm::vec3 position;
            glm::mat4 modelMatrix;
            glm::vec3 color;
        
            bool operator==(const ModelRenderInfo& other) const {
                return &model == &other.model && position == other.position;
            }
    };


    class Renderer {
    public:
        Renderer();

        void updateRays(const std::vector<std::vector<glm::vec3>>& trajectories);
        void render(glm::vec3 camPos, glm::vec3 camForward, const std::vector<ModelRenderInfo>& models);
        
    private:
        GLuint m_beamShaderProgram;
        GLuint m_beamShaderProgramMVPUniform;
        GLuint m_beamVAO, m_beamVBO;
        struct {
            GLuint count;
            GLuint primtCount;
            GLuint first;
            GLuint reserved;
        } m_beamDrawParams;

        GLuint m_modelShaderProgram;
        GLuint m_modelShaderProgramMVPUniform;
        GLuint m_modelShaderProgramModelUniform;
        GLuint m_modelShaderProgramColorUniform;
    };

} // namespace ElectronOptics