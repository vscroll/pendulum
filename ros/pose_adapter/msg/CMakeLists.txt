find_package(Protobuf REQUIRED)

set (msgs
  PendulumPose.proto
  VehiclePose.proto
  VehiclePendulumPose.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(pose_protobuf_msgs SHARED ${PROTO_SRCS})
target_link_libraries(pose_protobuf_msgs ${PROTOBUF_LIBRARY})
