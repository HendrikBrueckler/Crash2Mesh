if (MSVC)
    if(uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    set(ERR_WARN_FLAGS /sdl /W3)
    else()
    set(ERR_WARN_FLAGS /sdl /W3 /O2)
    endif()
else()
set(ERR_WARN_FLAGS -Wall -Wextra  -Wstrict-aliasing -pedantic -fmax-errors=5 -Werror
                    -Wunreachable-code -Wcast-align -Wcast-qual
                    -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op
                    -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual
                    -Wredundant-decls -Wshadow -Wsign-promo -Wstrict-null-sentinel
                    -Wstrict-overflow=2 -Wswitch-enum -Wundef -Wunused
                    -Wno-variadic-macros -Wno-parentheses -fdiagnostics-show-option)
endif()

### Find dependencies
find_package(OpenGL REQUIRED)

### Register all sources
file(GLOB_RECURSE SOURCE_LIST "*.cpp")
add_library(${PROJECT_NAME} ${SOURCE_LIST})

### Configurable options
option(C2M_PARALLEL "Whether to use parallel (threadsafe) algorithms where appropriate" OFF)

### Set compile options
if(${C2M_PARALLEL})
    add_compile_definitions(C2M_PARALLEL)
    message("Setting up using parallel algorithms!")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(${PROJECT_NAME} PRIVATE ${ERR_WARN_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/HighFive/include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/OpenMesh/src)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/Easy3D)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/extern/eigen)

### Link with dependencies
target_link_libraries(${PROJECT_NAME}
    HighFive
    OpenMeshCore
    OpenMeshTools
    ${OPENGL_LIBRARIES} glew glfw core viewer
)
