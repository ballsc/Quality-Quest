cmake_minimum_required(VERSION 3.15)

project(Serial_Test
    VERSION
        1.0
    LANGUAGES
        CXX
)


set(HEADERS 
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui.h
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/backends/imgui_impl_opengl3.h
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/backends/imgui_impl_glfw.h
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/backends/imgui_impl_opengl3_loader.h
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui_internal.h
    ${CMAKE_SOURCE_DIR}/src/serial_send.h
)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui.cpp
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/backends/imgui_impl_opengl3.cpp
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/extern/imgui-docking/imgui_widgets.cpp
)

add_executable(runTest
    # ${CMAKE_SOURCE_DIR}/src/gui.cpp
    ${CMAKE_SOURCE_DIR}/src/serial_comm.cpp
    # ${HEADERS}
    # ${SOURCES}
)