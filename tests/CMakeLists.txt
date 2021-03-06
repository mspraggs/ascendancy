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

add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")

set(SRC
  catch.hpp
  helpers.hpp
  TestRefGen.hpp

  helpers.cpp
  test_algorithms.cpp
  test_control_system.cpp
  test_controllers.cpp
  test_event_queue.cpp
  test_main.cpp
  test_ref_generators.cpp
  test_state_space.cpp
  test_serialisation.cpp)

add_executable(run_tests ${SRC})
target_link_libraries(run_tests ascendancy++ pthread)