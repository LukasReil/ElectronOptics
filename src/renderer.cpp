#include "renderer.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>


#include <fstream>
#include <sstream>

using namespace ElectronOptics;

static GLuint LoadShaders(const std::string& vertex_file_path, const std::string& fragment_file_path);

Renderer::Renderer() {

    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glEnable(GL_DEPTH_TEST);

    m_beamShaderProgram = LoadShaders("src/shaders/beamVertexShader.glsl", "src/shaders/beamFragmentShader.glsl");

    m_beamShaderProgramMVPUniform = glGetUniformLocation(m_beamShaderProgram, "MVP");

    glGenVertexArrays(1, &m_beamVAO);
    glGenBuffers(1, &m_beamVBO);

    m_modelShaderProgram = LoadShaders("src/shaders/modelVertexShader.glsl", "src/shaders/modelFragmentShader.glsl");
    m_modelShaderProgramMVPUniform = glGetUniformLocation(m_modelShaderProgram, "MVP");
    m_modelShaderProgramModelUniform = glGetUniformLocation(m_modelShaderProgram, "model");
    m_modelShaderProgramColorUniform = glGetUniformLocation(m_modelShaderProgram, "color");

}

void ElectronOptics::Renderer::updateRays(const std::vector<std::vector<glm::vec3>> &trajectories) {



    std::vector<glm::vec3> allPoints;
    allPoints.reserve(trajectories.size() * trajectories[0].size() * 2); // Reserve space to avoid reallocations
	for (const auto& traj : trajectories) {
		for (size_t i = 0; i < traj.size() - 1; ++i) {
			allPoints.push_back(traj[i]);
			allPoints.push_back(traj[i + 1]);
		}		
    }

	m_beamDrawParams.count = static_cast<GLuint>(allPoints.size());

	glBindVertexArray(m_beamVAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_beamVBO);
    glBufferData(GL_ARRAY_BUFFER, allPoints.size() * sizeof(glm::vec3), allPoints.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glVertexAttribDivisor(0, 0);
}

void ElectronOptics::Renderer::render(glm::vec3 camPos, glm::vec3 camForward, const std::vector<ModelRenderInfo>& models) {

    // glClearColor(0.05f,0.05f,0.08f,1);
    glClearColor(0.6f,0.6f,0.6f,1);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glUseProgram(m_beamShaderProgram);
    glBindVertexArray(m_beamVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_beamVBO);

    glm::mat4 view = glm::lookAt(camPos, camPos + camForward, glm::vec3(0,1,0));
    glm::ivec4 viewport;
    glGetIntegerv(GL_VIEWPORT, &viewport[0]);
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), static_cast<float>(viewport.z) / static_cast<float>(viewport.w), 0.1f, 100.0f);
    glm::mat4 mvp = projection * view;

    glUniformMatrix4fv(m_beamShaderProgramMVPUniform, 1, GL_FALSE, &mvp[0][0]);
	glBindVertexArray(m_beamVAO);
	glDrawArrays(GL_LINES, 0, m_beamDrawParams.count);

    // Render models
    glUseProgram(m_modelShaderProgram);
    glUniformMatrix4fv(m_modelShaderProgramMVPUniform, 1, GL_FALSE, &mvp[0][0]);
    for (const auto& modelInfo : models) {
        const Model& model = modelInfo.model;
        const glm::mat4& modelMatrix = modelInfo.modelMatrix;
        const glm::vec3& color = modelInfo.color;

        glUniformMatrix4fv(m_modelShaderProgramModelUniform, 1, GL_FALSE, &modelMatrix[0][0]);
        glUniform3fv(m_modelShaderProgramColorUniform, 1, &color[0]);

        // Render the model
        model.bind();
        glDrawArrays(GL_TRIANGLES, 0, model.getVertexCount());
    }

}

GLuint LoadShaders(const std::string& vertex_file_path, const std::string& fragment_file_path) {

	// Create the shaders
	GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
	GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

	// Read the Vertex Shader code from the file
	std::string VertexShaderCode;
	std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
	if(VertexShaderStream.is_open()){
		std::stringstream sstr;
		sstr << VertexShaderStream.rdbuf();
		VertexShaderCode = sstr.str();
		VertexShaderStream.close();
	}else{
		spdlog::error("Impossible to open {}. Are you in the right directory ? Don't forget to read the FAQ !", vertex_file_path);
		getchar();
		return 0;
	}

	// Read the Fragment Shader code from the file
	std::string FragmentShaderCode;
	std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
	if(FragmentShaderStream.is_open()){
		std::stringstream sstr;
		sstr << FragmentShaderStream.rdbuf();
		FragmentShaderCode = sstr.str();
		FragmentShaderStream.close();
	}else{ 
        spdlog::error("Impossible to open {}. Are you in the right directory ? Don't forget to read the FAQ !", fragment_file_path);
        getchar();
        return 0;
    }

	GLint Result = GL_FALSE;
	int InfoLogLength;

	// Compile Vertex Shader
	spdlog::info("Compiling shader : {}", vertex_file_path.c_str());
	char const * VertexSourcePointer = VertexShaderCode.c_str();
	glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
	glCompileShader(VertexShaderID);

	// Check Vertex Shader
	glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
		glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
		printf("%s\n", &VertexShaderErrorMessage[0]);
	}

	// Compile Fragment Shader
	spdlog::info("Compiling shader : {}", fragment_file_path.c_str());
	char const * FragmentSourcePointer = FragmentShaderCode.c_str();
	glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
	glCompileShader(FragmentShaderID);

	// Check Fragment Shader
	glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
		glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
		spdlog::error("Fragment shader compilation failed:\n{}", &FragmentShaderErrorMessage[0]);
	}

	// Link the program
	spdlog::info("Linking program");
	GLuint ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> ProgramErrorMessage(InfoLogLength+1);
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		spdlog::error("Program linking failed:\n{}", &ProgramErrorMessage[0]);
	}
	
	glDetachShader(ProgramID, VertexShaderID);
	glDetachShader(ProgramID, FragmentShaderID);
	
	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);

	return ProgramID;
}