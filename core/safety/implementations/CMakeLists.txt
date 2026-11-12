add_library(e_robot_arm.safety.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.safety.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.safety.implementations PUBLIC
    e_robot_arm.safety.interfaces
    numerical.math
)

target_sources(e_robot_arm.safety.implementations PRIVATE
    SafetySupervisorImpl.cpp
    SafetySupervisorImpl.hpp
)

add_subdirectory(test)
