# This file is part of Ascendancy++
#
# Copyright 2017 Matt Spraggs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Created by Matt Spraggs on 02/04/17

cmake_minimum_required(VERSION 2.8)
project(ascendancy++)

set(HEADERS
  Algorithm.hpp
  ConstRefGenerator.hpp
  Controller.hpp
  ControlSystem.hpp
  DataStore.hpp
  EventQueue.hpp
  GenericLiftedAlgorithm.hpp
  GenericRefGenerator.hpp
  globals.hpp
  NonCausalAlgorithm.hpp
  PIDController.hpp
  StateSpace.hpp
  RefGenerator.hpp
  ref_generators_generated.h
  serialisation.hpp
  utils.hpp)

set(SOURCES
  DataStore.cpp
  serialisation.cpp
  utils.cpp)

add_library(ascendancy++ SHARED ${HEADERS} ${SOURCES})
target_link_libraries(ascendancy++ pthread)

install(FILES ${HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ascendancy")
install(TARGETS ascendancy++ LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})