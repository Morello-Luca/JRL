set(controller_SRC
  DualArmControl.cpp
)

set(controller_HDR
  DualArmControl.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DDualArmControl_EXPORTS")

