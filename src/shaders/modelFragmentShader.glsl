#version 330 core

layout(location = 0) in vec3 vertexColor;
layout(location = 1) in vec3 vertexNormal;

out vec4 fragColor;

vec3 lightDir = normalize(vec3(0.0, 1.0, 0.5));

void main() {
    float brightness = dot(normalize(vertexNormal), lightDir);
    fragColor = vec4(vertexColor * clamp(brightness, 0.1, 1.0), 1.0);
}