### Find dependencies
find_package(HDF5 REQUIRED)

### Register all sources
file(GLOB_RECURSE SOURCE_LIST "*.cpp")
add_library(${PROJECT_NAME} ${SOURCE_LIST})

target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${HDF5_INCLUDE_DIRS})
### Link with dependencies
target_link_libraries(${PROJECT_NAME}
    HighFive
)
