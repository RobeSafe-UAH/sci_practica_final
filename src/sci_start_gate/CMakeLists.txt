cmake_minimum_required(VERSION 3.16)
project(sci_start_gate VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mvsim REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_topography REQUIRED)
find_package(box2d REQUIRED)
find_package(mvsim-msgs REQUIRED)
# MVSim 1.5's exported config still requests the old hyphenated MRPT package
# names. Import its official targets after finding the actual MRPT 3 packages;
# preserving their compile definitions is required for the public World ABI.
get_filename_component(MVSIM_PREFIX "${mvsim_DIR}/../../.." ABSOLUTE)
foreach(module comms simulator)
  find_file(MVSIM_${module}_TARGETS mvsim-${module}-targets.cmake HINTS
    "${MVSIM_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/mvsim-${module}"
    "${MVSIM_PREFIX}/lib/cmake/mvsim-${module}" REQUIRED)
  include(${MVSIM_${module}_TARGETS})
endforeach()
find_path(RAPIDXML_INCLUDE_DIR rapidxml/rapidxml.hpp REQUIRED)

add_library(sci_start_gate SHARED src/start_gate.cpp)
target_compile_features(sci_start_gate PRIVATE cxx_std_17)
target_compile_options(sci_start_gate PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(sci_start_gate PRIVATE include ${RAPIDXML_INCLUDE_DIR})
target_link_libraries(sci_start_gate PRIVATE mvsim::mvsim-simulator)
install(TARGETS sci_start_gate LIBRARY DESTINATION lib)

ament_package()
