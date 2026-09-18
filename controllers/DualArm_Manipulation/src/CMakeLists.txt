file(GLOB_RECURSE controller_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/controller/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tasks/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/filters/*.cpp"

)

file(GLOB_RECURSE controller_HDR CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/controller/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/tasks/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/filters/*.h"

)

add_controller(${PROJECT_NAME}
    ${controller_SRC}
    ${controller_HDR}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        CustomTask_EXPORTS
)
