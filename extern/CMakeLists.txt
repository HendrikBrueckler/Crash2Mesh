find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/HighFive/CMakeLists.txt"
    OR NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/OpenMesh/CMakeLists.txt"
    OR NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/Easy3D/CMakeLists.txt"
    OR NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/eigen/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

set(HIGHFIVE_EXAMPLES FALSE)
set(HIGHFIVE_UNIT_TESTS FALSE)
set(HIGHFIVE_USE_BOOST FALSE)
set(HIGHFIVE_PARALLEL_HDF5 FALSE)
set(HIGHFIVE_DOCS FALSE)
add_subdirectory(HighFive)

set(OPENMESH_DOCS FALSE)
add_subdirectory(OpenMesh)

set(EASY3D_BUILD_TUTORIALS OFF)
set(WITH_GFLAGS OFF)
if(uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
add_definitions(-DNDEBUG)
add_subdirectory(Easy3D)
remove_definitions(-DNDEBUG)
else()
add_subdirectory(Easy3D)
endif()

if (MSVC)
    if(uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
        file( COPY Easy3D/resources DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug )
    else()
        file( COPY Easy3D/resources DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release )
    endif()
else()
    file( COPY Easy3D/resources DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
endif()