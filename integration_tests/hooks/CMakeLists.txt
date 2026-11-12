add_library(e_robot_arm.integration_tests.hooks OBJECT
    Hooks.cpp
)

target_link_libraries(e_robot_arm.integration_tests.hooks PUBLIC
    cucumber_cpp.runner
)
