cmake_minimum_required(VERSION 3.16)

project(crocoddyl_plotter CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
find_package(OpenMP REQUIRED)
find_package(crocoddyl REQUIRED)

add_subdirectory(proto)
add_subdirectory(python)
add_subdirectory(client-app)

add_library(crocoddyl_plotter SHARED src/mim_solvers_callbacks.cpp)
target_include_directories(crocoddyl_plotter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(crocoddyl_plotter PUBLIC crocoddyl_plotter_grpc_proto crocoddyl::crocoddyl)
install(TARGETS crocoddyl_plotter DESTINATION lib)