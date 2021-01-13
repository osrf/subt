##############################################################################
# Find Protobuf & Custom Macros
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Protobuf.cmake)

##############################################################################
# Generate Messages
##############################################################################

set(PROTO_MESSAGES
  datagram.proto
  neighbor_m.proto
)

protobuf_generate_cpp_with_descriptor(
  PROTO_SOURCES
  PROTO_HEADERS
  PROTO_DESCRIPTORS
  IMPORT_DIRS
    ${ignition-msgs6_INCLUDE_DIRS}
  DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
  ${PROTO_MESSAGES}
)

# Variables needed to propagate through modules
# If more than one layer of cmake use CACHE INTERNAL ...
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SOURCES ${PROTO_SOURCES} PARENT_SCOPE)
set(PROTO_HEADERS ${PROTO_HEADERS} PARENT_SCOPE)

add_custom_target(${PROJECT_NAME}_protobuf_compilation ALL DEPENDS ${PROTO_SOURCES})
