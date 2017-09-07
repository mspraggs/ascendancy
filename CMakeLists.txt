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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Eigen3 3.0 REQUIRED)

include_directories(lib ${EIGEN3_INCLUDE_DIR})

include(GNUInstallDirs)

set(CMAKE_CXX_FLAGS
  "-g -Wall -Wextra -Wnon-virtual-dtor -pedantic -std=c++11 ${CMAKE_CXX_FLAGS}")

add_subdirectory(lib)
add_subdirectory(benchmarks)
add_subdirectory(tests)