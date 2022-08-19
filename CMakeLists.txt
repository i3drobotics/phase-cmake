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
    file (GLOB PHASE_LIBRARY_FILES
        "${PHASE_LIB_DIR}/*.so*"
    )
    file (GLOB PHASE_I3DRSGM_LIBRARY_FILES
        "${PHASE_LIB_DIR}/i3drsgm/*.so*"
    )
endif()

# Define GNU standard installation directories
include(GNUInstallDirs)

set(LIB_SRC_FILES
    src/runphase.cpp
)

# Define app source files
set(APP_SRC_FILES
    src/main.cpp
)

# Build lib
add_library(runphase ${LIB_SRC_FILES})
target_link_libraries(runphase ${Phase_LIBS})
target_include_directories(runphase PRIVATE
    include
    ${Phase_INCLUDE_DIRS}
)
target_compile_definitions(runphase PRIVATE ${Phase_DEFINITIONS})
# avoid seperate 'release' and 'debug' folders
set_target_properties(runphase PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib
)
# Install app in bin folder
install(TARGETS runphase
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Build app
add_executable(${PROJECT_NAME_LOWER} ${APP_SRC_FILES})
target_link_libraries(${PROJECT_NAME_LOWER} runphase ${Phase_LIBS})
target_include_directories(${PROJECT_NAME_LOWER} PRIVATE
    include
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

if (UNIX)
    add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
        COMMAND mkdir -p $<TARGET_FILE_DIR:${PROJECT_NAME}>/i3drsgm)
    foreach(PHASE_LIBRARY_FILEPATH ${PHASE_I3DRSGM_LIBRARY_FILES})
        get_filename_component(PHASE_LIBRARY_FILE "${PHASE_LIBRARY_FILEPATH}" NAME)
        get_filename_component(PHASE_LIBRARY_FILE_EXT "${PHASE_LIBRARY_FILEPATH}" EXT)
        # copy library file to target directory
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
                        COMMAND cp -a -u
                        ${PHASE_LIBRARY_FILEPATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/i3drsgm/)
    endforeach()
endif()

foreach(PHASE_LIBRARY_FILEPATH ${PHASE_LIBRARY_FILES})
    get_filename_component(PHASE_LIBRARY_FILE "${PHASE_LIBRARY_FILEPATH}" NAME)
    get_filename_component(PHASE_LIBRARY_FILE_EXT "${PHASE_LIBRARY_FILEPATH}" EXT)
    # copy library file to target directory
    if (WIN32)
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy 
                           ${PHASE_LIBRARY_FILEPATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    elseif (UNIX)
        add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
                           COMMAND cp -a -u
                           ${PHASE_LIBRARY_FILEPATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>)
        set(PHASE_LIBRARY_FILEPATH_FULL 
            ${CMAKE_BINARY_DIR}/bin/${PHASE_LIBRARY_FILE}
        )
        # change rpath for dynamic library to find dependencies in local directory
        set(origin_var "\\$$ORIGIN")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND if ! [ -L ${PHASE_LIBRARY_FILEPATH_FULL} ] \; then patchelf --set-rpath ${origin_var} ${PHASE_LIBRARY_FILEPATH_FULL} \; fi
        )
    endif()
endforeach()

if (UNIX)
    set(origin_var "\\$$ORIGIN:\\$$ORIGIN/i3drsgm")
    # change rpath for dynamic library to find dependencies in local directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND patchelf --set-rpath ${origin_var}
        ${CMAKE_BINARY_DIR}/bin/libphase.so
    )
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND patchelf --set-rpath ${origin_var}
        $<TARGET_FILE:${PROJECT_NAME}>
    )
endif()
