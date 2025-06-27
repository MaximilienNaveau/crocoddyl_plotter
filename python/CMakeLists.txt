find_package(Python3 REQUIRED)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "-c" "import sys, os; print(os.sep.join(['lib', 'python' + '.'.join(sys.version.split('.')[:2]), 'site-packages']))"
    OUTPUT_VARIABLE PYTHON_SITELIB
    OUTPUT_STRIP_TRAILING_WHITESPACE
    #ERROR_QUIET
)
message(STATUS "Python site lib: ${PYTHON_SITELIB}")

set(PROTOC_INPUT_DIR ${CMAKE_SOURCE_DIR}/proto)

set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
file(MAKE_DIRECTORY ${OUTPUT_DIR})
foreach(pyfile crocoddyl_plotter/__init__.py)
    file(COPY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}" ${OUTPUT_DIR}/${pyfile})
    list(APPEND REPO_PY_FILES ${OUTPUT_DIR}/${pyfile})
endforeach()

set(PROTOC_PY_FILES "${OUTPUT_DIR}/crocoddyl_plotter/crocoddyl_pb2.py" "${OUTPUT_DIR}/crocoddyl_plotter/crocoddyl_pb2_grpc.py")

add_custom_command(
      OUTPUT ${PROTOC_PY_FILES}
      COMMAND "${Python3_EXECUTABLE}"
      ARGS -m grpc_tools.protoc
        --python_out "${OUTPUT_DIR}"
        --grpc_python_out "${OUTPUT_DIR}"
        -Icrocoddyl_plotter="${PROTOC_INPUT_DIR}"
        "${PROTOC_INPUT_DIR}/crocoddyl.proto"
      DEPENDS "${PROTOC_INPUT_DIR}/crocoddyl.proto")
message(STATUS "Python site lib: ${Python3_EXECUTABLE}")
add_custom_target(protoc_python ALL DEPENDS ${PROTOC_PY_FILES})

set(PY_NAME ${PROJECT_NAME}_pywrap)
set(${PY_NAME}_HEADERS utils/callback-base.hpp)
set(${PY_NAME}_SOURCES
    main.cpp
    mim-solvers-callbacks.cpp)
add_library(${PY_NAME} MODULE ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME} PUBLIC crocoddyl_plotter eigenpy::eigenpy)


set_target_properties(
  ${PY_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME}
             PREFIX ""
             SUFFIX ${PYTHON_EXT_SUFFIX})

set(PYLIB_INSTALL_DIR ${PYTHON_SITELIB}/crocoddyl_plotter)
# if(UNIX)
#   get_relative_rpath(${PYLIB_INSTALL_DIR} PYLIB_RPATH)
#   set_target_properties(${PY_NAME} PROPERTIES INSTALL_RPATH ${PYLIB_RPATH})
# endif()

install(TARGETS ${PY_NAME} DESTINATION ${PYLIB_INSTALL_DIR})
install(FILES ${REPO_PY_FILES} ${PROTOC_PY_FILES} DESTINATION ${PYLIB_INSTALL_DIR})