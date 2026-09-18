cmake_minimum_required(VERSION 3.16)

project(my_box)

find_package(mc_rtc REQUIRED)

set(MY_BOX_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/my_box")
set(MY_BOX_DESCRIPTION_PATH "${MY_BOX_SHARE_DIR}")

configure_file(
    src/config.in.h
    "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

set(MY_BOX_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/my_box.cpp"
)

set(MY_BOX_HDR
    "${CMAKE_CURRENT_SOURCE_DIR}/src/my_box.h"
)

add_robot(my_box
    ${MY_BOX_SRC}
    ${MY_BOX_HDR}
)

target_include_directories(
    my_box
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_BINARY_DIR}"
)

install(
    DIRECTORY data/urdf
    DESTINATION ${MY_BOX_SHARE_DIR}
)

install(
    DIRECTORY data/meshes
    DESTINATION ${MY_BOX_SHARE_DIR}
)

install(
    DIRECTORY data/convex
    DESTINATION ${MY_BOX_SHARE_DIR}
)

install(
    DIRECTORY data/rsdf
    DESTINATION ${MY_BOX_SHARE_DIR}
)

install(
    FILES data/my_box.xml
    DESTINATION ${MY_BOX_SHARE_DIR}
)

install(
    FILES data/my_box.yaml
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/mc_mujoco
)