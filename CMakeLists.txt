cmake_minimum_required(VERSION 3.21)

set(ENGINE_NAME "ElectronOptics")

project(${ENGINE_NAME})
set(CMAKE_CXX_STANDARD 23)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)

add_executable(${ENGINE_NAME} ${SRC_FILES})
target_include_directories(${ENGINE_NAME} PRIVATE src)


add_subdirectory(lib)