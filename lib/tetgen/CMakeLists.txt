cmake_minimum_required(VERSION 3.21)

add_library(tetgen STATIC
    tetgen.cxx 
    predicates.cxx
)

target_include_directories(tetgen PUBLIC ./)