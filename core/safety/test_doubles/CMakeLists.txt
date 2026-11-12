add_library(e_robot_arm.safety.test_doubles INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.safety.test_doubles INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(e_robot_arm.safety.test_doubles INTERFACE
    e_robot_arm.safety.interfaces
    gmock
)

target_sources(e_robot_arm.safety.test_doubles INTERFACE
    SafetySupervisorMock.hpp
)
