add_library(e_robot_arm.motion.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.motion.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.motion.implementations PUBLIC
    e_robot_arm.motion.interfaces
    infra.timer
    numerical.math
    numerical.motion_profiles
)

target_sources(e_robot_arm.motion.implementations PRIVATE
    JointMotionImpl.cpp
    JointMotionImpl.hpp
    JointMovePlan.cpp
    JointMovePlan.hpp
)

add_subdirectory(test)
