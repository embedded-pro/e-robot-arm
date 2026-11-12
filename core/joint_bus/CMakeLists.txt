add_subdirectory(interfaces)
add_subdirectory(implementations)

if (E_ROBOT_ARM_BUILD_TESTS)
    add_subdirectory(test_doubles)
endif()
