cmake_minimum_required(VERSION 3.21)

include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

if (WIN32)
    set(GLFW_BUILD_X11 OFF)
    set(GLFW_BUILD_WAYLAND OFF)
endif()

FetchContent_MakeAvailable(glfw)