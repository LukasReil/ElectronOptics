cmake_minimum_required(VERSION 3.21)

include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.1
)

FetchContent_MakeAvailable(eigen)
