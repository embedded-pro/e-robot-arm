add_library(e_robot_arm.tool.virtual_arm.plant_port STATIC)

target_include_directories(e_robot_arm.tool.virtual_arm.plant_port PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_sources(e_robot_arm.tool.virtual_arm.plant_port PRIVATE
    MotorConnection.cpp
    MotorConnection.hpp
    PlantPortCodec.cpp
    PlantPortCodec.hpp
)

add_subdirectory(test)
