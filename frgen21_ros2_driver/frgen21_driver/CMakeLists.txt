cmake_minimum_required(VERSION 3.8)
project(frgen21_driver VERSION 1.2.2)

# Ensure the CMake build type is set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include GNUInstallDirs for standard installation directories
include(GNUInstallDirs)

# Find Boost libraries
find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread regex)

# Add library
add_library(${PROJECT_NAME} SHARED
    src/udp_com.cpp
    src/R01.00/PointTargetInterfaceMapping.cpp
    src/R01.00/R1PhysicalValueDerivation.cpp
    src/R01.00/frgen21_frame_reassembler_ZX_R01.00.cpp
    src/R01.00/frgen21_frame_interpreter_ZX_R01.00.cpp
    src/R01.00/frgen21_input_signal_handler_R01.00.cpp
    src/R02.00/frgen21_input_signal_handler_R02.00.cpp
    src/R02.00/PointTargetInterfaceMapping.cpp
    src/R02.00/R2PhysicalValueDerivation.cpp
    src/R02.00/frgen21_frame_interpreter_ZX_R02.00.cpp
    src/R02.02/frgen21_input_signal_handler_R02.02.cpp
    src/R02.50/frgen21_frame_reassembler_ZX_R02.50.cpp    
    src/frgen21_frame_reassembler.cpp
    src/frgen21_frame_interpreter.cpp
    src/frgen21_input_signal_handler.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
        ${Boost_INCLUDE_DIRS}
)

# Link Boost libraries
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

# Install the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Export the targets to a script
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Export the package
export(EXPORT ${PROJECT_NAME}Targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
)

# Create the Config version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Configure the Config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Install the Config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Change permissions of the installed directories
install(CODE "execute_process(COMMAND chmod -R o+rx ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake)")
