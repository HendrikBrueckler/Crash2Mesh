set(EXECUTABLE_NAME "${PROJECT_NAME}_pretests")

add_executable(${EXECUTABLE_NAME} main.cpp)

target_include_directories(${EXECUTABLE_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/HighFive/include)
target_include_directories(${EXECUTABLE_NAME} SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/OpenVolumeMesh/src)

target_link_libraries(${EXECUTABLE_NAME} crash2mesh)
