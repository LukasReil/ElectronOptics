cmake_minimum_required(VERSION 3.21)

add_subdirectory(glfw)
add_subdirectory(glad)
add_subdirectory(glm)
add_subdirectory(spdlog)
find_package(OpenMP REQUIRED)
add_subdirectory(eigen)

target_link_libraries(${ENGINE_NAME} PUBLIC 
    glfw
    glad
    glm
)

target_link_libraries(${ENGINE_NAME} PRIVATE
    spdlog
    eigen
    OpenMP::OpenMP_CXX
)