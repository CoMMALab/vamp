# Python bindings configuration for VAMP
# This file contains all Python bindings related logic

# The python name a robot registers itself under (Robot::name) normally matches its
# module/file name, but not always -- IiwaMarker::name is "iiwamarker" (no underscore)
# while its header/module is iiwa_marker. Every place that touches the *runtime* python
# attribute (the robots() list, stub generation/install) needs this; the generated .cc
# file, init_<x>() function, and #include keep using the raw module name instead.
#
# This and vamp_robot_extra_binder() below are looked up by name rather than kept as
# lists parallel to VAMP_ROBOT_MODULES/STRUCTS on purpose: VAMP_ROBOT_MODULES can also be
# set from outside this file (pyproject.toml's scikit-build cmake.define ships a curated
# robot subset for the wheel), which knows nothing about such lists -- keeping several
# lists in lockstep by construction is exactly the kind of thing that silently desyncs.
function(vamp_robot_py_name robot_name out_var)
  if(robot_name STREQUAL "iiwa_marker")
    set(${out_var} "iiwamarker" PARENT_SCOPE)
  else()
    set(${out_var} "${robot_name}" PARENT_SCOPE)
  endif()
endfunction()

# Robot-specific extra #include/call spliced into its generated robot.cc.in (see
# VAMP_ROBOT_EXTRA_INCLUDE/VAMP_ROBOT_EXTRA_CALL_LINE below), for binders too
# robot-specific to live in the generic robot_helper.hh (e.g. rby1_sampler_binder.hh's
# FixedBaseSampler, which hardcodes RBY1::ParameterizedSpace's State layout). Empty
# strings for every other robot.
function(vamp_robot_extra_binder robot_name out_header out_call)
  if(robot_name STREQUAL "rby1")
    set(${out_header} "vamp/bindings/python/rby1_sampler_binder.hh" PARENT_SCOPE)
    # No trailing ';': robot.cc.in supplies the terminating ';' itself.
    set(${out_call} "vamp::binding::bind_rby1_task_sampler(submodule)" PARENT_SCOPE)
  else()
    set(${out_header} "" PARENT_SCOPE)
    set(${out_call} "" PARENT_SCOPE)
  endif()
endfunction()

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

  if(NOT VAMP_ROBOT_MODULES)
    list(APPEND VAMP_ROBOT_MODULES
      sphere
      ur5
      panda
      bimanual_panda
      fetch
      baxter
      digit
      r2c6
    )

    list(APPEND VAMP_ROBOT_STRUCTS
      Sphere
      UR5
      Panda
      BimanualPanda
      Fetch
      Baxter
      Digit
      R2c6
    )

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/robots/rby1.hh")
      list(APPEND VAMP_ROBOT_MODULES rby1)
      list(APPEND VAMP_ROBOT_STRUCTS RBY1)
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/robots/iiwa_marker.hh")
      list(APPEND VAMP_ROBOT_MODULES iiwa_marker)
      list(APPEND VAMP_ROBOT_STRUCTS IiwaMarker)
    endif()
  endif()

  foreach(robot IN LISTS VAMP_ROBOT_MODULES)
    vamp_robot_py_name(${robot} py_name)
    string(APPEND VAMP_ROBOT_INITS "    vb::init_${robot}(pymodule);\n")
    string(APPEND VAMP_ROBOT_DECLS "    void init_${robot}(nanobind::module_ &pymodule);\n")
    string(APPEND VAMP_ROBOT_QUOTED "\"${py_name}\",")
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

  if(VAMP_BUILD_JIT)
    list(APPEND VAMP_EXT_SOURCES
      src/impl/vamp/bindings/python/dynamic.cc
    )
  endif()

  foreach(robot_name robot_struct IN ZIP_LISTS VAMP_ROBOT_MODULES VAMP_ROBOT_STRUCTS)
  vamp_robot_extra_binder(${robot_name} extra_header extra_call)
  set(VAMP_ROBOT_EXTRA_INCLUDE "")
  if(extra_header)
    set(VAMP_ROBOT_EXTRA_INCLUDE "#include <${extra_header}>")
  endif()
  set(VAMP_ROBOT_EXTRA_CALL_LINE "${extra_call}")

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

  if(VAMP_BUILD_JIT)
    target_link_libraries(_core_ext PRIVATE
        vamp::jit
        cricket::cricket
    )
    target_compile_definitions(_core_ext PRIVATE VAMP_HAVE_JIT=1)

    set_property(TARGET _core_ext APPEND PROPERTY INSTALL_RPATH
        "$<TARGET_FILE_DIR:cricket::cricket>"
        "$<TARGET_FILE_DIR:cricket::cricket_jit>"
        "$ORIGIN/../../cricket/lib"
    )

    # simdxorshift symbols need exported so dynamic symbol search finds them
    if(TARGET simdxorshift)
      target_link_options(_core_ext PRIVATE
          "LINKER:--whole-archive,$<TARGET_FILE:simdxorshift>,--no-whole-archive"
          "LINKER:--export-dynamic"
      )
      add_dependencies(_core_ext simdxorshift)
    endif()
  endif()

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
    vamp_robot_py_name(${robot_name} py_name)
    nanobind_add_stub(
      "vamp_${robot_name}_stub"
      MODULE "_core_ext.${py_name}"
      OUTPUT "${STUB_PREFIX}${py_name}.pyi"
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
    vamp_robot_py_name(${robot_name} py_name)
    install(
      FILES "${STUB_PREFIX}${py_name}.pyi"
      DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
    )
  endforeach()
endif()
