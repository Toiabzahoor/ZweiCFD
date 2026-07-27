cmake_minimum_required(VERSION 3.10)

project(ZweiFoil CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    rlImGui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(rlImGui)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(imgui)

set(RLIMGUI_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/rlimgui-src")
set(IMGUI_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/imgui-src")
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/_deps/rlimgui-src/rlImGui.cpp")
    set(RLIMGUI_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/rlimgui-src")
elseif(DEFINED rlImGui_SOURCE_DIR AND EXISTS "${rlImGui_SOURCE_DIR}/rlImGui.cpp")
    set(RLIMGUI_SOURCE_DIR "${rlImGui_SOURCE_DIR}")
elseif(DEFINED rlimgui_SOURCE_DIR AND EXISTS "${rlimgui_SOURCE_DIR}/rlImGui.cpp")
    set(RLIMGUI_SOURCE_DIR "${rlimgui_SOURCE_DIR}")
endif()
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/_deps/imgui-src/imgui.cpp")
    set(IMGUI_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/imgui-src")
elseif(DEFINED imgui_SOURCE_DIR AND EXISTS "${imgui_SOURCE_DIR}/imgui.cpp")
    set(IMGUI_SOURCE_DIR "${imgui_SOURCE_DIR}")
elseif(DEFINED imgui_SOURCE_DIR AND EXISTS "${imgui_SOURCE_DIR}/imgui.h")
    set(IMGUI_SOURCE_DIR "${imgui_SOURCE_DIR}")
endif()

include_directories(include)

add_library(imgui_static STATIC
    ${IMGUI_SOURCE_DIR}/imgui.cpp
    ${IMGUI_SOURCE_DIR}/imgui_draw.cpp
    ${IMGUI_SOURCE_DIR}/imgui_tables.cpp
    ${IMGUI_SOURCE_DIR}/imgui_widgets.cpp
)
target_include_directories(imgui_static PUBLIC ${IMGUI_SOURCE_DIR})

add_library(rlImGui_static STATIC
    ${RLIMGUI_SOURCE_DIR}/rlImGui.cpp
)
target_include_directories(rlImGui_static PUBLIC
    ${RLIMGUI_SOURCE_DIR}
    ${IMGUI_SOURCE_DIR}
)
target_link_libraries(rlImGui_static PUBLIC raylib imgui_static)

add_executable(ZweiFoil
    src/main.cpp
    src/airfoil.cpp
    src/solver.cpp
)
target_include_directories(ZweiFoil PRIVATE
    ${raylib_SOURCE_DIR}/src
)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

target_link_libraries(ZweiFoil PRIVATE raylib imgui_static rlImGui_static)


if(MINGW OR WIN32)
    target_link_options(ZweiFoil PRIVATE -static -static-libgcc -static-libstdc++)
endif()
