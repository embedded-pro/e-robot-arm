add_library(e_robot_arm.example.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.example.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.example.interfaces INTERFACE
    infra.util
)

target_sources(e_robot_arm.example.interfaces PRIVATE
    Accumulator.hpp
)
