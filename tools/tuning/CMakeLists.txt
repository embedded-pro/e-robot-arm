add_library(e_robot_arm.tool.tuning STATIC)

target_include_directories(e_robot_arm.tool.tuning PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(e_robot_arm.tool.tuning PUBLIC
    e_robot_arm.tool.arm_host
    numerical.math
)

target_sources(e_robot_arm.tool.tuning PRIVATE
    TuningMetrics.cpp
    TuningMetrics.hpp
    TuningRun.cpp
    TuningRun.hpp
)

add_subdirectory(test)
