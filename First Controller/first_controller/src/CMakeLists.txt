set(controller_SRC
  Ur5Manipulation.cpp
)

set(controller_HDR
  Ur5Manipulation.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DUr5Manipulation_EXPORTS")

