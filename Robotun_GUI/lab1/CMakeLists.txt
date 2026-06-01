cmake_minimum_required(VERSION 3.14)
project(lab1 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 1. Fetch GLFW (Windowing and Input)
message(STATUS "Fetching GLFW...")
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# 2. Fetch Dear ImGui (UI)
message(STATUS "Fetching Dear ImGui...")
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.8
)
FetchContent_MakeAvailable(imgui)

# Set source directories
set(IMGUI_DIR ${imgui_SOURCE_DIR})

# Gather Dear ImGui source files
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)


# Define the Executable
add_executable(lab1
    src/main.cpp
    ${IMGUI_SOURCES}
)

# Add necessary compile definitions for OpenGL3
target_compile_definitions(lab1 PRIVATE
    IMGUI_IMPL_OPENGL_LOADER_GLEW
)

# Setup include paths
target_include_directories(lab1 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# Link standard system libraries (OpenGL)
if(WIN32)
    set(OPENGL_LIBRARIES opengl32)
elseif(APPLE)
    find_library(OPENGL_LIBRARIES OpenGL REQUIRED)
else()
    find_package(OpenGL REQUIRED)
    set(OPENGL_LIBRARIES ${OPENGL_LIBRARIES})
endif()

target_link_libraries(lab1 PRIVATE
    glfw
    ${OPENGL_LIBRARIES}
)

message(STATUS "lab1 configuration complete!")
