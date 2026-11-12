add_library(e_robot_arm.example.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.example.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.example.implementations PUBLIC
    e_robot_arm.example.interfaces
)

target_sources(e_robot_arm.example.implementations PRIVATE
    AccumulatorImpl.hpp
    AccumulatorImpl.cpp
)

add_subdirectory(test)
