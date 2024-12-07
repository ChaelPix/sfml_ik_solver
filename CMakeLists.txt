cmake_minimum_required(VERSION 3.0)
project(pixx_ik)

find_package(TGUI 1.2.0 REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} TGUI::TGUI sfml-graphics sfml-window sfml-system)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/ressources DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/ressources)
