include(GNUInstallDirs)

set(component tools)
set(resources "${CMAKE_CURRENT_LIST_DIR}/resources")
set(E_ROBOT_ARM_PACKAGE_ENGINE "" CACHE FILEPATH "e-foc-virtual-motor to ship in the tools package")

foreach(program e_robot_arm.tool.simulator e_robot_arm.tool.arm_console e_robot_arm.tool.virtual_arm.main)
    if (TARGET ${program})
        install(TARGETS ${program} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${component} EXCLUDE_FROM_ALL)
    endif()
endforeach()

if (E_ROBOT_ARM_PACKAGE_ENGINE)
    install(PROGRAMS "${E_ROBOT_ARM_PACKAGE_ENGINE}" DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${component} EXCLUDE_FROM_ALL)
endif()

if (NOT WIN32)
    install(FILES e-robot-arm-simulator.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${resources}/e-robot-arm-simulator.svg" DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${resources}/e-robot-arm-simulator.png" DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/256x256/apps COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/e-robot-arm RENAME copyright COMPONENT ${component} EXCLUDE_FROM_ALL)
endif()

function(e_robot_arm_tools_package)
    set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackConfig.cmake")
    set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackSourceConfig.cmake")
    set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/packages")
    set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};e_robot_arm;${component};/")
    set(CPACK_MONOLITHIC_INSTALL ON)
    set(CPACK_VERBATIM_VARIABLES ON)
    set(CPACK_GENERATOR "DEB")

    set(CPACK_PACKAGE_NAME "e-robot-arm")
    set(CPACK_PACKAGE_VENDOR "embedded-pro")
    set(CPACK_PACKAGE_VERSION "${e_robot_arm_VERSION}")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "e-robot-arm Simulator and tools for a six-joint arm on e-foc motor nodes")
    set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/embedded-pro/e-robot-arm")
    set(CPACK_PACKAGE_FILE_NAME "e_robot_arm-tools-v${e_robot_arm_VERSION}-${CMAKE_SYSTEM_NAME}")
    set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
    set(CPACK_STRIP_FILES ON)

    set(CPACK_DEBIAN_PACKAGE_NAME "e-robot-arm")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "embedded-pro")
    set(CPACK_DEBIAN_PACKAGE_SECTION "electronics")
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "qt6-qpa-plugins, qml6-module-qtqml, qml6-module-qtqml-workerscript, qml6-module-qtquick, qml6-module-qtquick-window, qml6-module-qtquick3d | qml6-module-quick3d, qml6-module-qtquick3d-assetutils | qml6-module-quick3d-assetutils, qml6-module-qtquick3d-helpers | qml6-module-quick3d-helpers")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
        "Simulator and console for a six-joint robot arm on e-foc motor nodes.\n Drives the arm on simulated joints, on e-foc virtual motors (included) or on\n the real arm through a CAN adapter.")

    include(CPack)
endfunction()

e_robot_arm_tools_package()
