project(KidnappedVehicle)
cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++0x)

add_executable(KidnappedVehicle
  src/main.cc
  src/utils.cc
  src/particle_filter.cc)
