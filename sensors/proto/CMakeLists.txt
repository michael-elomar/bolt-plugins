find_program(PROTOBUF_PROTOC protoc REQUIRED)

message(STATUS "Protobuf found")

set(PROTOBUF_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set(PROTOBUF_OUTPUT "${CMAKE_BINARY_DIR}/generated/proto")

set_property(GLOBAL APPEND PROPERTY PROTO_CC_HEADERS ${PROTOBUF_OUTPUT})

set(PROTO_SRC
	gnss.proto
)

if(NOT EXISTS "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-c")
	file(MAKE_DIRECTORY "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-c")
endif()

if(NOT EXISTS "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-cc")
	file(MAKE_DIRECTORY "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-cc")
endif()

set(PROTOBUF_C_OUTPUT "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-c")
set(PROTOBUF_CXX_OUTPUT "${PROTOBUF_OUTPUT}/${PROJECT_NAME}-cc")


# set the arguments for the protobuf compiler
set (protoc_args
  "--cpp_out=${PROTOBUF_CXX_OUTPUT}"
  "--c_out=${PROTOBUF_C_OUTPUT}"
  "-I${PROTOBUF_PATH}"
  "${PROTO_SRC}"
)

execute_process(
	COMMAND ${PROTOBUF_PROTOC} ${protoc_args}
	RESULT_VARIABLE CMD_RESULT
	OUTPUT_VARIABLE CMD_OUTPUT
	ERROR_VARIABLE CMD_ERROR
)

if(NOT CMD_RESULT EQUAL 0)
    message(FATAL_ERROR
		"Protobuf compiler failed with exit code ${CMD_RESULT}\n"
        "stdout:\n${CMD_OUTPUT}\n"
        "stderr:\n${CMD_ERROR}"
    )
endif()

file(GLOB C_SRC_FILES
    CONFIGURE_DEPENDS
	"${PROTOBUF_C_OUTPUT}/*.c"
)

message(STATUS ${C_SRC_FILES})

file(GLOB CC_SRC_FILES
    CONFIGURE_DEPENDS
	"${PROTOBUF_CXX_OUTPUT}/*.cc"
)

message(STATUS ${CC_SRC_FILES})

add_library(${PROJECT_NAME}-pb-c SHARED ${C_SRC_FILES})
target_include_directories(${PROJECT_NAME}-pb-c PRIVATE $<BUILD_INTERFACE:${PROTOBUF_C_OUTPUT}>)
target_link_libraries(${PROJECT_NAME}-pb-c protobuf::libprotobuf)

add_library(${PROJECT_NAME}-pb-cc SHARED ${CC_SRC_FILES})
target_include_directories(${PROJECT_NAME}-pb-cc PRIVATE $<BUILD_INTERFACE:${PROTOBUF_CXX_OUTPUT}>)
target_link_libraries(${PROJECT_NAME}-pb-cc protobuf::libprotobuf)

#======== Install Support ========#

set(PROTOBUF_INSTALL_TARGETS
	${PROJECT_NAME}-pb-c
	${PROJECT_NAME}-pb-cc
)

install(TARGETS ${PROTOBUF_INSTALL_TARGETS}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

