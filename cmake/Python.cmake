# Python bindings configuration for VAMP
# This file contains all Python bindings related logic

if(VAMP_BUILD_PYTHON_BINDINGS)
  # Find Python and nanobind dependencies
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

  if(NOT Python_FOUND)
    message(FATAL_ERROR "VAMP_BUILD_PYTHON_BINDINGS is ON but Python was not found")
  endif()

  find_package(nanobind CONFIG QUIET)
  if(NOT nanobind_FOUND)
    CPMAddPackage("gh:wjakob/nanobind#9a25aed8a7edfe60ef9ad1c911e57667bc4916c4")
  endif()

  # Robots compiled into the Python module. VAMP_ROBOTS is a ;-separated list of robot names, or "all".
  # Every robot adds a large header to the build, so pip builds (pyproject.toml) only ask for a few;
  # a direct CMake build with neither variable set gets everything. Setting VAMP_ROBOT_MODULES together
  # with VAMP_ROBOT_STRUCTS still works and takes precedence.
  set(VAMP_KNOWN_ROBOTS
    sphere=Sphere
    ur5=UR5
    panda=Panda
    bimanual_panda=BimanualPanda
    fetch=Fetch
    baxter=Baxter
    digit=Digit
    r2c6=R2c6
    bimanual_iiwa=BimanualIiwa
    g1_unitree=G1Unitree
  )

  if(NOT VAMP_ROBOT_MODULES)
    if(NOT VAMP_ROBOTS)
      set(VAMP_ROBOTS "all")
    endif()

    set(_vamp_requested ${VAMP_ROBOTS})
    if(VAMP_ROBOTS STREQUAL "all")
      set(_vamp_requested "")
      foreach(_entry ${VAMP_KNOWN_ROBOTS})
        string(REGEX REPLACE "=.*$" "" _name "${_entry}")
        list(APPEND _vamp_requested "${_name}")
      endforeach()
    endif()

    foreach(_robot ${_vamp_requested})
      set(_struct "")
      foreach(_entry ${VAMP_KNOWN_ROBOTS})
        if(_entry MATCHES "^${_robot}=(.*)$")
          set(_struct "${CMAKE_MATCH_1}")
        endif()
      endforeach()

      if(_struct STREQUAL "")
        message(FATAL_ERROR "VAMP_ROBOTS: unknown robot '${_robot}'. Known: ${VAMP_KNOWN_ROBOTS} (name=Struct)")
      endif()

      list(APPEND VAMP_ROBOT_MODULES ${_robot})
      list(APPEND VAMP_ROBOT_STRUCTS ${_struct})
    endforeach()
  endif()

  message(STATUS "Python robot modules: ${VAMP_ROBOT_MODULES}")

  foreach(robot ${VAMP_ROBOT_MODULES})
    string(APPEND VAMP_ROBOT_INITS "    vb::init_${robot}(pymodule);\n")
    string(APPEND VAMP_ROBOT_DECLS "    void init_${robot}(nanobind::module_ &pymodule);\n")
    string(APPEND VAMP_ROBOT_QUOTED "\"${robot}\",")
  endforeach()

  list(JOIN VAMP_ROBOT_QUOTED ", " VAMP_ROBOT_NAMES)

  configure_file(
    src/impl/vamp/bindings/python/init.hh.in
    ${CMAKE_CURRENT_BINARY_DIR}/vamp_python_init.hh
    @ONLY
  )

  configure_file(
    src/impl/vamp/bindings/python/python.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    src/impl/vamp/bindings/python/environment.cc
    src/impl/vamp/bindings/python/settings.cc
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
  )


  foreach(robot_name robot_struct IN ZIP_LISTS VAMP_ROBOT_MODULES VAMP_ROBOT_STRUCTS)
  configure_file(
    src/impl/vamp/bindings/python/robot.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
  )
  endforeach()

  nanobind_add_module(_core_ext
    NB_STATIC
    STABLE_ABI
    NOMINSIZE
    ${VAMP_EXT_SOURCES}
  )

  target_include_directories(_core_ext
    SYSTEM PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(_core_ext
    PRIVATE
    vamp_cpp
    Eigen3::Eigen
  )


  if($ENV{GITHUB_ACTIONS})
    set(STUB_PREFIX "")
  else()
    set(STUB_PREFIX "${CMAKE_BINARY_DIR}/stubs/")
  endif()

  # Disable strict warnings for Python bindings to maintain compatibility
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(_core_ext PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
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
