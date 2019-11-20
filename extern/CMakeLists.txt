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
set(HIGHFIVE_DOCS false)
add_subdirectory(HighFive)

set(OPENMESH_DOCS false)
add_subdirectory(OpenMesh)
set (OPENMESH_FOUND ${OPENMESH_FOUND} PARENT_SCOPE)
set (OPENMESH_LIBRARIES ${OPENMESH_LIBRARIES} PARENT_SCOPE)
set (OPENMESH_INCLUDE_DIR  ${OPENMESH_INCLUDE_DIR} PARENT_SCOPE)
set (OPENMESH_INCLUDE_DIRS ${OPENMESH_INCLUDE_DIRS} PARENT_SCOPE)

set (OPENMESH_CORE_LIBRARY ${OPENMESH_CORE_LIBRARY} PARENT_SCOPE)
set (OPENMESH_TOOLS_LIBRARY ${OPENMESH_TOOLS_LIBRARY} PARENT_SCOPE)
set (OPENMESH_LIBRARY ${OPENMESH_LIBRARY} PARENT_SCOPE)
set (_OPENMESH_LIBRARY_DIR ${_OPENMESH_LIBRARY_DIR} PARENT_SCOPE)
set (OPENMESH_LIBRARY_DIR "${_OPENMESH_LIBRARY_DIR}" CACHE PATH "The directory where the OpenMesh libraries can be found.")
