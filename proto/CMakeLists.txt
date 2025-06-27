# Generated sources
# set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
# set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)
find_program(_PROTOBUF_PROTOC protoc)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
set(PROTOC_INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTOC_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(crocoddyl_proto_srcs "${PROTOC_OUTPUT_DIR}/crocoddyl.pb.cc")
set(crocoddyl_proto_hdrs "${PROTOC_OUTPUT_DIR}/crocoddyl.pb.h")
set(crocoddyl_grpc_srcs "${PROTOC_OUTPUT_DIR}/crocoddyl.grpc.pb.cc")
set(crocoddyl_grpc_hdrs "${PROTOC_OUTPUT_DIR}/crocoddyl.grpc.pb.h")
add_custom_command(
      OUTPUT "${crocoddyl_proto_srcs}" "${crocoddyl_proto_hdrs}" "${crocoddyl_grpc_srcs}" "${crocoddyl_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${PROTOC_OUTPUT_DIR}"
        --cpp_out "${PROTOC_OUTPUT_DIR}"
        -I "${PROTOC_INPUT_DIR}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${PROTOC_INPUT_DIR}/crocoddyl.proto"
      DEPENDS "${PROTOC_INPUT_DIR}/crocoddyl.proto")


# crocoddyl_plotter_grpc_proto
add_library(crocoddyl_plotter_grpc_proto
  SHARED
  ${crocoddyl_grpc_srcs}
  ${crocoddyl_grpc_hdrs}
  ${crocoddyl_proto_srcs}
  ${crocoddyl_proto_hdrs})
target_include_directories(crocoddyl_plotter_grpc_proto PUBLIC $<BUILD_INTERFACE:${PROTOC_OUTPUT_DIR}>)
target_link_libraries(crocoddyl_plotter_grpc_proto PUBLIC
  ${_REFLECTION}
  gRPC::grpc++
  protobuf::libprotobuf)
install(TARGETS crocoddyl_plotter_grpc_proto DESTINATION lib)