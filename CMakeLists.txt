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

# Get Phase library files
set(PHASE_ROOT_DIR "${Phase_DIR}/../..")
get_filename_component(PHASE_ROOT_DIR "${PHASE_ROOT_DIR}" ABSOLUTE)
set(PHASE_BIN_DIR 
    "${PHASE_ROOT_DIR}/bin"
)
set(PHASE_LIB_DIR 
    "${PHASE_ROOT_DIR}/lib"
)
if (WIN32)
    file (GLOB PHASE_LIBRARY_FILES
        "${PHASE_BIN_DIR}/*.dll"
    )
elseif(UNIX)
    file (GLOB PHASE_CORE_LIBRARY_FILES
        "${PHASE_LIB_DIR}/*.so"
        "${PHASE_LIB_DIR}/*.so*"
    )
    file (GLOB PHASE_I3DRSGM_LIBRARY_FILES
        "${PHASE_LIB_DIR}/i3drsgm/*.so"
        "${PHASE_LIB_DIR}/i3drsgm/*.so*"
    )
    set (PHASE_LIBRARY_FILES
        ${PHASE_CORE_LIBRARY_FILES}
        ${PHASE_I3DRSGM_LIBRARY_FILES}
    )
endif()

# Install Phase libraries
set(rpath_origin "\\$$ORIGIN")
if (UNIX)
    foreach(PHASE_LIBRARY_FILEPATH ${PHASE_LIBRARY_FILES})
        get_filename_component(PHASE_LIBRARY_FILE "${PHASE_LIBRARY_FILEPATH}" NAME)
        get_filename_component(PHASE_LIBRARY_FILE_EXT "${PHASE_LIBRARY_FILEPATH}" EXT)
        # copy library file to target directory
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
                        COMMAND cp -a -u
                        ${PHASE_LIBRARY_FILEPATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/)
        # change rpath for dynamic library to find dependencies in local directory
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
            COMMAND patchelf --set-rpath ${rpath_origin} $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PHASE_LIBRARY_FILE}
        )
    endforeach()
endif()

if (UNIX)
    # change rpath to find dependencies in local directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND patchelf --set-rpath ${rpath_origin}
        $<TARGET_FILE:${PROJECT_NAME}>
    )
endif()