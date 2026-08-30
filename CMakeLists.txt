cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)

option(VAMP_BUILD_PYTHON_BINDINGS "Build VAMP Python bindings" ON)
option(VAMP_INSTALL_CPP_LIBRARY "Install VAMP C++ library (disable for Python wheel builds)" ON)

option(VAMP_BUILD_CPP_DEMO "Build VAMP C++ Demo Scripts" OFF)
option(VAMP_BUILD_OMPL_DEMO "Build VAMP C++ OMPL Integration Demo Scripts" OFF)
option(VAMP_OMPL_PATH "Search Path for OMPL Installation - Only Needed for Demo Script" "")
option(VAMP_BUILD_WASM "Build VAMP WebAssembly module" OFF)
option(VAMP_BUILD_JIT "Build the vamp::jit runtime" OFF)

# Opt-in kernel profiling (steer/validate/IK/collision/NN/sample timing + call counts) for
# the C++ demo scripts. Off by default: no VAMP_PROFILING define means the
# VAMP_PROFILE_SCOPE() instrumentation in src/impl/vamp/planning compiles to nothing, so this
# flag does not affect normal builds at all. Only affects VAMP_BUILD_CPP_DEMO targets.
option(VAMP_ENABLE_PROFILING "Build C++ demo scripts with kernel profiling instrumentation" OFF)

if(VAMP_FORCE_CLANG)
  find_program(CLANG "clang")
  find_program(CLANGPP "clang++")

  set(CMAKE_C_COMPILER ${CLANG} CACHE STRING "C compiler" FORCE)
  set(CMAKE_CXX_COMPILER ${CLANGPP} CACHE STRING "C++ compiler" FORCE)
endif()

if(VAMP_FORCE_COLORED_OUTPUT)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
  endif()
endif()

project(
    vamp
    VERSION 0.2
    LANGUAGES C CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(macros)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CompilerSettings)
include(FetchInitCPM)
include(Dependencies)

# Create VAMP C++ library
add_library(vamp_cpp INTERFACE)

# Set required C++ standard for the library
target_compile_features(vamp_cpp INTERFACE cxx_std_17)

# vamp::Vector aliases rows in place as smaller Vector types via reinterpret_cast
# (vector/interface.hh row()), which violates strict aliasing; GCC 16 -O2 dead-store
# eliminates whole-Vector writes because of it. Propagate the flag to all consumers.
target_compile_options(vamp_cpp INTERFACE -fno-strict-aliasing)

target_include_directories(vamp_cpp
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/vamp>
)

target_link_libraries_system(vamp_cpp
    INTERFACE
    Eigen3::Eigen
    pdqsort
)

# Link SIMDxorshift if available
if(TARGET simdxorshift)
  target_link_libraries(vamp_cpp INTERFACE simdxorshift)
endif()

# Set library properties
set_target_properties(vamp_cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Create alias target for modern CMake usage
add_library(vamp::vamp ALIAS vamp_cpp)

# Build JIT
include(JIT)

# Build Python bindings
include(Python)

# Build WASM bindings
include(WASM)

# Install C++ library components (conditional for Python wheel builds)
if(VAMP_INSTALL_CPP_LIBRARY)
  # Install C++ library headers and SIMD library (if available)
  set(VAMP_INSTALL_TARGETS vamp_cpp)
  if(TARGET simdxorshift)
    list(APPEND VAMP_INSTALL_TARGETS simdxorshift)
  endif()

  install(TARGETS ${VAMP_INSTALL_TARGETS}
      EXPORT vamp_cppTargets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Install headers
  install(DIRECTORY src/impl/vamp/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vamp
      FILES_MATCHING PATTERN "*.hh"
  )

  # Generate package configuration files
  configure_package_config_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vampConfig.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfig.cmake"
      INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )

  write_basic_package_version_file(
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfigVersion.cmake"
      VERSION ${PROJECT_VERSION}
      COMPATIBILITY SameMajorVersion
  )

  # Install package configuration files
  install(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfig.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfigVersion.cmake"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )

  # Export targets
  install(EXPORT vamp_cppTargets
      FILE vamp_cppTargets.cmake
      NAMESPACE vamp::
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )
endif()

# C++ demos
if(VAMP_BUILD_CPP_DEMO)
  add_executable(vamp_rrtc_example scripts/cpp/rrtc_example.cc)
  target_link_libraries(vamp_rrtc_example PRIVATE vamp_cpp)

  # Disable strict warnings for demos to maintain compatibility
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(vamp_rrtc_example PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/robots/rby1.hh")
    CPMAddPackage("gh:nlohmann/json@3.11.3")

    add_executable(vamp_rby1_task_space_planner scripts/cpp/rby1_task_space_planner.cc)
    target_link_libraries(vamp_rby1_task_space_planner PRIVATE vamp_cpp nlohmann_json::nlohmann_json)

    add_executable(vamp_rby1_gcp_branch_selector scripts/cpp/rby1_gcp_branch_selector.cc)
    target_link_libraries(vamp_rby1_gcp_branch_selector PRIVATE vamp_cpp)

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(vamp_rby1_task_space_planner PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
      target_compile_options(vamp_rby1_gcp_branch_selector PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()

    if(VAMP_ENABLE_PROFILING)
      target_compile_definitions(vamp_rby1_task_space_planner PRIVATE VAMP_PROFILING)
    endif()
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/robots/iiwa_marker.hh")
    CPMAddPackage("gh:nlohmann/json@3.11.3")

    add_executable(vamp_iiwa_maze_solver scripts/cpp/iiwa_maze_solver.cc)
    target_link_libraries(vamp_iiwa_maze_solver PRIVATE vamp_cpp nlohmann_json::nlohmann_json)

    add_executable(vamp_iiwa_maze_solver_benchmark scripts/cpp/iiwa_maze_solver_benchmark.cc)
    target_link_libraries(vamp_iiwa_maze_solver_benchmark PRIVATE vamp_cpp nlohmann_json::nlohmann_json)

    add_executable(vamp_iiwa_branch_selector scripts/cpp/iiwa_branch_selector.cc)
    target_link_libraries(vamp_iiwa_branch_selector PRIVATE vamp_cpp nlohmann_json::nlohmann_json)

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(vamp_iiwa_maze_solver PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
      target_compile_options(vamp_iiwa_maze_solver_benchmark PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
      target_compile_options(vamp_iiwa_branch_selector PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()

    if(VAMP_ENABLE_PROFILING)
      target_compile_definitions(vamp_iiwa_maze_solver_benchmark PRIVATE VAMP_PROFILING)
      target_compile_definitions(vamp_iiwa_maze_solver PRIVATE VAMP_PROFILING)
    endif()
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/robots/bimanual_iiwa.hh")
    add_executable(vamp_bimanual_iiwa_leader_follower_shelf
      scripts/cpp/bimanual_iiwa_leader_follower_shelf.cc)
    target_link_libraries(vamp_bimanual_iiwa_leader_follower_shelf PRIVATE vamp_cpp)

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(vamp_bimanual_iiwa_leader_follower_shelf PRIVATE
        -Wno-c++11-narrowing -Wno-sign-compare)
    endif()

    if(VAMP_ENABLE_PROFILING)
      target_compile_definitions(vamp_bimanual_iiwa_leader_follower_shelf PRIVATE VAMP_PROFILING)
    endif()
  endif()
endif()

# OMPL integration demo
if(VAMP_BUILD_OMPL_DEMO)
  find_package(ompl QUIET PATHS ${VAMP_OMPL_PATH})

  if(ompl_FOUND)
    add_executable(vamp_ompl_integration scripts/cpp/ompl_integration.cc)
    target_link_libraries(vamp_ompl_integration PRIVATE ompl::ompl vamp_cpp)
    
    # Disable strict warnings for demos to maintain compatibility
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(vamp_ompl_integration PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()
  else()
    message(WARNING "OMPL not found! Cannot build OMPL demo.")
  endif()
endif()


# Print build configuration
message(STATUS "VAMP build configuration:")
message(STATUS "  - C++ library: ON")
message(STATUS "  - Install C++ library: ${VAMP_INSTALL_CPP_LIBRARY}")
message(STATUS "  - Python bindings: ${VAMP_BUILD_PYTHON_BINDINGS}")
message(STATUS "  - JIT: ${VAMP_BUILD_JIT}")
message(STATUS "  - C++ demos: ${VAMP_BUILD_CPP_DEMO}")
message(STATUS "  - C++ demo profiling: ${VAMP_ENABLE_PROFILING}")
message(STATUS "  - OMPL demos: ${VAMP_BUILD_OMPL_DEMO}")
message(STATUS "  - WASM module: ${VAMP_BUILD_WASM}")
