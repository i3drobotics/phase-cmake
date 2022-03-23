cmake_minimum_required(VERSION 3.9)

if (NOT (WIN32 OR UNIX OR APPLE))
    message(FATAL_ERROR "Only Windows, Linux and Apple currently supported")
endif ()

project(phase_sample LANGUAGES CXX)

# Define names
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# Find Phase
set(Phase_DIR $ENV{Phase_DIR} CACHE STRING "Phase root directory")
find_package(Phase REQUIRED)

# Define GNU standard installation directories
include(GNUInstallDirs)

# Define app source files
set(APP_SRC_FILES
    src/main.cpp
)

# Build app
add_executable(${PROJECT_NAME_LOWER} ${APP_SRC_FILES})
target_link_libraries(${PROJECT_NAME_LOWER} ${Phase_LIBS})
target_include_directories(${PROJECT_NAME_LOWER} PRIVATE
    include
    ${Phase_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME_LOWER} PRIVATE ${Phase_DEFINITIONS})
# avoid seperate 'release' and 'debug' folders
set_target_properties(${PROJECT_NAME_LOWER} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib
)

# Install app in bin folder
install(TARGETS ${PROJECT_NAME_LOWER}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)