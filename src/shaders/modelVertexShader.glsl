#version 330 core

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;

uniform mat4 MVP;
uniform mat4 model;
uniform vec3 color;

layout(location = 0) out vec3 vertexColor;
layout(location = 1) out vec3 vertexNormal;

void main() {
    gl_Position = MVP * model * vec4(pos, 1.0);
    vertexColor = color;
    vertexNormal = (model * vec4(normal, 0.0)).xyz;
}