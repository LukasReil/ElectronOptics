# Electron Optics Simualtor

This project is an electron optics simulator that uses FEM for calculating the electric field inside a bounded volume and traces electrons through the field to determine the effects of different lens properties.

## Usage

This tool is currently not intended to be used by others, but in its current state it reads `.obj` files and produces an electron path and the potential field, both of which are written to `.txt` files (look at `src/main.cpp`).

## Dependencies

* [Eigen](https://gitlab.com/libeigen/eigen)
* [GLFW](https://github.com/glfw/glfw)
* [GLAD](https://github.com/Dav1dde/glad)
* [GLM](https://github.com/g-truc/glm)
* [spdlog](https://github.com/gabime/spdlog)
* [tetgen](https://github.com/TetGen/TetGen)

## Planned features

* B-Field integration
* E-Beam parameter analysis
* 3d Visualization