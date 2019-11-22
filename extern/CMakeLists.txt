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
    OR NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/OpenMesh/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

set(HIGHFIVE_EXAMPLES FALSE)
set(HIGHFIVE_UNIT_TESTS FALSE)
set(USE_BOOST TRUE)
set(HIGHFIVE_PARALLEL_HDF5 FALSE)
set(HIGHFIVE_DOCS FALSE)
add_subdirectory(HighFive)

set(OPENMESH_DOCS FALSE)
add_subdirectory(OpenMesh)

set(EASY3D_BUILD_TUTORIALS FALSE)
add_subdirectory(Easy3D)

file( COPY ${CMAKE_BINARY_DIR}/resource DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
