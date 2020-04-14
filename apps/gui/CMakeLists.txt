set(EXECUTABLE_NAME "${PROJECT_NAME}_gui")

add_executable(${EXECUTABLE_NAME} main.cpp)

target_link_libraries(${EXECUTABLE_NAME} crash2mesh)
