add_executable(e_robot_arm.example_app)

target_include_directories(e_robot_arm.example_app PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.example_app PUBLIC
    infra.util
    e_robot_arm.example.implementations
)

target_sources(e_robot_arm.example_app PRIVATE
    Main.cpp
)
