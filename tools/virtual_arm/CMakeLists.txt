add_library(e_robot_arm.tool.virtual_arm STATIC)

target_include_directories(e_robot_arm.tool.virtual_arm PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.virtual_arm PUBLIC
    e_robot_arm.arm
    e_robot_arm.tool.virtual_arm.plant_port
)

target_sources(e_robot_arm.tool.virtual_arm PRIVATE
    ArmCoupling.cpp
    ArmCoupling.hpp
    VirtualArmPlan.cpp
    VirtualArmPlan.hpp
)

add_subdirectory(plant_port)
add_subdirectory(test)
