add_library(e_robot_arm.tool.simulator.samples STATIC)

target_include_directories(e_robot_arm.tool.simulator.samples PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.tool.simulator.samples PUBLIC
    e_robot_arm.tool.arm_host
    ui.stage
)

target_sources(e_robot_arm.tool.simulator.samples PRIVATE
    GltfWriter.cpp
    GltfWriter.hpp
    SampleArm.cpp
    SampleArm.hpp
)

add_executable(e_robot_arm.tool.simulator.sample_meshes)
set_target_properties(e_robot_arm.tool.simulator.sample_meshes PROPERTIES OUTPUT_NAME e-robot-arm-sample-meshes)
target_link_libraries(e_robot_arm.tool.simulator.sample_meshes PRIVATE e_robot_arm.tool.simulator.samples)
target_sources(e_robot_arm.tool.simulator.sample_meshes PRIVATE main/Main.cpp)

add_subdirectory(test)
