add_executable(e_robot_arm.tool.example)

target_include_directories(e_robot_arm.tool.example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.tool.example PUBLIC
    infra.util
    e_robot_arm.example.implementations
)

target_sources(e_robot_arm.tool.example PRIVATE
    Main.cpp
)
