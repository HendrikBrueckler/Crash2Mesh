project(${PROJECT_NAME}_pretests)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} crash2mesh)