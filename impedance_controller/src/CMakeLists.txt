set(controller_SRC
  Impedance.cpp
)

set(controller_HDR
  Impedance.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DImpedance_EXPORTS")

