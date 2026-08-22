cmake_minimum_required(VERSION 3.10)

project(ZweiCFD CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

include(FetchContent)


include_directories(include)

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    GUISupportQt
    FiltersModeling
    FiltersFlowPaths
    IOGeometry
)

add_executable(ZweiCFD
    src/core/main.cpp
    src/solver/airfoil.cpp
    src/solver/solver.cpp
    src/core/simulation.cpp
    src/solver/inviscid_lvpm.cpp
    src/solver/lbm_solver.cpp
    src/core/config.cpp
    src/ui/main_window.cpp
    include/ZweiCFD/ui/main_window.hpp
    src/ui/controls_dialog.cpp
    include/ZweiCFD/ui/controls_dialog.hpp
)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()

target_include_directories(ZweiCFD PRIVATE
    ${eigen_SOURCE_DIR}
    ${VTK_INCLUDE_DIRS}
)

target_compile_options(ZweiCFD PRIVATE -fopenmp)
target_link_libraries(ZweiCFD PRIVATE ${VTK_LIBRARIES} Qt6::Widgets)

if(MINGW OR WIN32)
    target_link_options(ZweiCFD PRIVATE -fopenmp -static -static-libgcc -static-libstdc++)
endif()






add_custom_command(TARGET ZweiCFD POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/assets"
    "$<TARGET_FILE_DIR:ZweiCFD>/assets"
    VERBATIM
)

# Copy all required DLL dependencies to the build output directory
add_custom_command(TARGET ZweiCFD POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:ZweiCFD> $<TARGET_FILE_DIR:ZweiCFD>
    COMMAND_EXPAND_LISTS
    VERBATIM
)

# Run windeployqt to bundle Qt plugins and dependencies
find_program(WINDEPLOYQT_EXECUTABLE windeployqt)
if(WINDEPLOYQT_EXECUTABLE)
    add_custom_command(TARGET ZweiCFD POST_BUILD
        COMMAND "${WINDEPLOYQT_EXECUTABLE}" --compiler-runtime "$<TARGET_FILE:ZweiCFD>"
        COMMENT "Running windeployqt to copy Qt dependencies..."
    )
endif()
