set(controller_SRC
  Test.cpp
)

set(controller_HDR
  Test.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DTest_EXPORTS")

