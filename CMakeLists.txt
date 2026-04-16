cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)
option(VAMP_BUILD_PYTHON "Force the use of Clang." ON)

option(VAMP_BUILD_JAX "Build JAX Extensions." OFF)
option(VAMP_USE_OPENMP "Use OpenMP." ON)
option(XLA_DIR "JAX FFI Directory" "")

option(VAMP_BUILD_PYTHON_BINDINGS "Build VAMP Python bindings" ON)
option(VAMP_INSTALL_CPP_LIBRARY "Install VAMP C++ library (disable for Python wheel builds)" ON)

option(VAMP_BUILD_CPP_DEMO "Build VAMP C++ Demo Scripts" OFF)
option(VAMP_BUILD_OMPL_DEMO "Build VAMP C++ OMPL Integration Demo Scripts" OFF)
option(VAMP_OMPL_PATH "Search Path for OMPL Installation - Only Needed for Demo Script" "")

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
# set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
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

find_package(Eigen3 REQUIRED NO_MODULE)

if(VAMP_USE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
set(NIGH_INCLUDE_DIRS ${nigh_SOURCE_DIR}/src)

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
set(PDQSORT_INCLUDE_DIRS ${pdqsort_SOURCE_DIR})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
  list(APPEND VAMP_SOURCES
    ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
    ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c)
  list(APPEND VAMP_INCLUDES ${SIMDxorshift_SOURCE_DIR}/include)
endif()

list(APPEND VAMP_INCLUDES
  ${NIGH_INCLUDE_DIRS}
  ${PDQSORT_INCLUDE_DIRS}
  src/impl
)

if(VAMP_BUILD_PYTHON)
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

  CPMAddPackage("gh:wjakob/nanobind#358d452c314dbe8c07026d984ad8d5aa860f26fb")

  if(NOT VAMP_ROBOT_MODULES)
    list(APPEND VAMP_ROBOT_MODULES
      sphere
      ur5
      panda
      fetch
      baxter
    )

    list(APPEND VAMP_ROBOT_STRUCTS
      Sphere
      UR5
      Panda
      Fetch
      Baxter
    )
  endif()

  foreach(robot ${VAMP_ROBOT_MODULES})
    string(APPEND VAMP_ROBOT_INITS "    vb::init_${robot}(pymodule);\n")
    string(APPEND VAMP_ROBOT_DECLS "    void init_${robot}(nanobind::module_ &pymodule);\n")
    string(APPEND VAMP_ROBOT_QUOTED "\"${robot}\",")
  endforeach()

  list(JOIN VAMP_ROBOT_QUOTED ", " VAMP_ROBOT_NAMES)

  configure_file(
    src/impl/vamp/bindings/init.hh.in
    ${CMAKE_CURRENT_BINARY_DIR}/vamp_python_init.hh
    @ONLY
  )

  configure_file(
    src/impl/vamp/bindings/python.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    src/impl/vamp/bindings/environment.cc
    src/impl/vamp/bindings/settings.cc
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
  )

  foreach(robot_name robot_struct IN ZIP_LISTS VAMP_ROBOT_MODULES VAMP_ROBOT_STRUCTS)
    configure_file(
      src/impl/vamp/bindings/robot.cc.in
      ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
      @ONLY
    )

    list(APPEND VAMP_EXT_SOURCES
      ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
    )
  endforeach()

  if(VAMP_BUILD_JAX)
      list(APPEND VAMP_ROBOT_JAX_MODULES
        panda
        fetch
      )

      list(APPEND VAMP_EXT_SOURCES "src/impl/vamp/bindings/jax/jax.cc")

      foreach(robot_name IN LISTS VAMP_ROBOT_JAX_MODULES)
        list(APPEND VAMP_EXT_SOURCES "src/impl/vamp/bindings/jax/${robot_name}.cc")
      endforeach()

      add_definitions(-DVAMP_BUILD_JAX)
  endif()

  if(VAMP_USE_OPENMP)
    add_definitions(-DVAMP_USE_OPENMP)
  endif()

  nanobind_add_module(_core_ext
    NB_STATIC
    STABLE_ABI
    NOMINSIZE
    ${VAMP_EXT_SOURCES}
    ${VAMP_SOURCES}
  )

  target_include_directories(_core_ext
    SYSTEM PRIVATE
    ${VAMP_INCLUDES}
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(_core_ext
    PRIVATE
    Eigen3::Eigen
  )

  if(VAMP_BUILD_JAX)
    target_include_directories(_core_ext PRIVATE ${XLA_DIR})
  endif()

  if(VAMP_USE_OPENMP)
    target_link_libraries(_core_ext
      PRIVATE OpenMP::OpenMP_CXX
    )
  endif()

  if($ENV{GITHUB_ACTIONS})
    set(STUB_PREFIX "")
  else()
    set(STUB_PREFIX "${CMAKE_BINARY_DIR}/stubs/")
  endif()

  nanobind_add_stub(
    vamp_stub
    MODULE _core_ext
    OUTPUT "${STUB_PREFIX}__init__.pyi"
    PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
    DEPENDS _core_ext
    VERBOSE
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    nanobind_add_stub(
      "vamp_${robot_name}_stub"
      MODULE "_core_ext.${robot_name}"
      OUTPUT "${STUB_PREFIX}${robot_name}.pyi"
      PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
      DEPENDS _core_ext
      VERBOSE
    )
  endforeach()

  install(
    TARGETS _core_ext
    LIBRARY
    DESTINATION vamp/_core
  )

  install(
    FILES "${STUB_PREFIX}__init__.pyi"
    DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    install(
      FILES "${STUB_PREFIX}${robot_name}.pyi"
      DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
    )
  endforeach()
endif()

# Create VAMP C++ library
add_library(vamp_cpp INTERFACE)

# Set required C++ standard for the library
target_compile_features(vamp_cpp INTERFACE cxx_std_17)

target_include_directories(vamp_cpp
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/vamp>
)

target_link_libraries_system(vamp_cpp
    INTERFACE
  Eigen3::Eigen
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

# Python bindings are already configured above in the VAMP_BUILD_PYTHON block.
# Avoid re-including cmake/Python.cmake here, which would create duplicate
# nanobind targets such as _core_ext and vamp_*_stub.

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
message(STATUS "  - C++ demos: ${VAMP_BUILD_CPP_DEMO}")
message(STATUS "  - OMPL demos: ${VAMP_BUILD_OMPL_DEMO}")
