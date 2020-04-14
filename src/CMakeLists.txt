if (MSVC)
    if(uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    set(C2M_COMPILE_FLAGS /sdl /W3 /fp:fast)
    else()
    set(C2M_COMPILE_FLAGS /sdl /W3 /O2 /fp:fast)
    endif()
else()
set(C2M_COMPILE_FLAGS -ffast-math -Wall -Wextra  -Wstrict-aliasing -pedantic -fmax-errors=5 -Werror
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
option(C2M_PROB_QUADRICS "Whether to use probabilistic instead of standard quadrics" OFF)

set(C2M_DEPENDENCIES
    HighFive
    OpenMeshCore
    OpenMeshTools
    ${OPENGL_LIBRARIES}
    glew
    glfw
    core
    gui
    util
    viewer
    imgui
)

### Set compile options
if(${C2M_PARALLEL})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "9.0")
            find_package(TBB)
            if (TBB_FOUND)
            message("Setting up using parallel algorithms!")
            add_compile_definitions(C2M_PARALLEL)
            set(C2M_DEPENDENCIES ${C2M_DEPENDENCIES} TBB::tbb)
            endif()
        endif()
    else()
        message("Setting up using parallel algorithms!")
        add_compile_definitions(C2M_PARALLEL)
    endif()
endif()

if (${C2M_PROB_QUADRICS})
    message("Setting up using probabilistic quadrics!")
    add_compile_definitions(C2M_PROB_QUADRICS)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(${PROJECT_NAME} PRIVATE ${C2M_COMPILE_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/HighFive/include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/OpenMesh/src)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/extern/Easy3D)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../extern/Easy3D)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/extern/eigen)

### Link with dependencies
target_link_libraries(${PROJECT_NAME} ${C2M_DEPENDENCIES})
