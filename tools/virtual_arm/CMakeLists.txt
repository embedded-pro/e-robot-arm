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

# The launcher runtime starts processes and opens sockets with POSIX calls: Linux first (virtual-arm.md).
if (UNIX)
    target_sources(e_robot_arm.tool.virtual_arm PRIVATE
        MotorProcess.cpp
        MotorProcess.hpp
        PlantPortListener.cpp
        PlantPortListener.hpp
        VirtualArm.cpp
        VirtualArm.hpp
    )
    target_link_libraries(e_robot_arm.tool.virtual_arm PRIVATE can_lite.drivers.loopback)

    add_executable(e_robot_arm.tool.virtual_arm.main)
    set_target_properties(e_robot_arm.tool.virtual_arm.main PROPERTIES OUTPUT_NAME e-robot-arm-virtual-arm)

    target_link_libraries(e_robot_arm.tool.virtual_arm.main PRIVATE
        e_robot_arm.tool.virtual_arm
        e_robot_arm.tool.arm_host
    )

    target_sources(e_robot_arm.tool.virtual_arm.main PRIVATE
        main/Main.cpp
    )
endif()

add_subdirectory(plant_port)
add_subdirectory(test)
