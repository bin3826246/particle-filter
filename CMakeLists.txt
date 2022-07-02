project(particle)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

file(GLOB HEADERS src/*.h)
#file(GLOB HEADERS_HPP src/*.hpp)

set(sources src/particle_filter.cpp src/main.cpp ${HEADERS})

add_executable(particle_filter ${sources})
