set(ERR_WARN_FLAGS -Wall -Wextra  -Wstrict-aliasing -pedantic -fmax-errors=5 -Werror
                    -Wunreachable-code -Wcast-align -Wcast-qual
                    -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op
                    -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual
                    -Wredundant-decls -Wshadow -Wsign-promo -Wstrict-null-sentinel
                    -Wstrict-overflow=5 -Wswitch-enum -Wundef -Wno-unused
                    -Wno-variadic-macros -Wno-parentheses -fdiagnostics-show-option)

### Find dependencies

### Register all sources
file(GLOB_RECURSE SOURCE_LIST "*.cpp")
add_library(${PROJECT_NAME} ${SOURCE_LIST})

### Set compile options
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_compile_options(${PROJECT_NAME} PRIVATE ${ERR_WARN_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/extern/HighFive/include)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/extern/OpenVolumeMesh/src)

### Link with dependencies
target_link_libraries(${PROJECT_NAME}
    HighFive
    OpenVolumeMesh
)
