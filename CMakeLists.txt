cmake_minimum_required(VERSION 3.10)

project(ZweiCFD CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets REQUIRED)

include(FetchContent)

include_directories(include)

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    CommonTransforms
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    GUISupportQt
    FiltersCore
    FiltersGeneral
    FiltersModeling
    FiltersFlowPaths
    IOGeometry
    IOXML
    RenderingVolume
    RenderingVolumeOpenGL2
)

add_executable(ZweiCFD
    src/core/main.cpp
    src/core/cli.cpp
    include/ZweiCFD/core/cli.hpp
    src/solver/airfoil.cpp
    src/solver/solver.cpp
    src/core/simulation.cpp
    src/solver/inviscid_lvpm.cpp
    src/solver/lbm_solver.cpp
    src/core/config.cpp
    src/render/gpu_advection.cpp
    include/ZweiCFD/render/gpu_advection.hpp
    src/ui/main_window.cpp
    include/ZweiCFD/ui/main_window.hpp
    src/ui/controls_dialog.cpp
    include/ZweiCFD/ui/controls_dialog.hpp
    src/ui/polar_dialog.cpp
    include/ZweiCFD/ui/polar_dialog.hpp
)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
    )
    FetchContent_MakeAvailable(eigen)
endif()

if(APPLE)
    find_package(OpenMP)
    if(NOT TARGET OpenMP::OpenMP_CXX)
        find_path(OpenMP_INCLUDE_DIR omp.h HINTS /opt/homebrew/opt/libomp/include /usr/local/opt/libomp/include)
        find_library(OpenMP_LIBRARY omp HINTS /opt/homebrew/opt/libomp/lib /usr/local/opt/libomp/lib)
        if(OpenMP_INCLUDE_DIR AND OpenMP_LIBRARY)
            add_library(OpenMP::OpenMP_CXX INTERFACE IMPORTED)
            set_target_properties(OpenMP::OpenMP_CXX PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${OpenMP_INCLUDE_DIR}"
                INTERFACE_LINK_LIBRARIES "${OpenMP_LIBRARY}"
                INTERFACE_COMPILE_OPTIONS "-Xpreprocessor;-fopenmp"
            )
        endif()
    endif()
else()
    find_package(OpenMP REQUIRED)
endif()

target_include_directories(ZweiCFD PRIVATE
    ${VTK_INCLUDE_DIRS}
)

target_link_libraries(ZweiCFD PRIVATE ${VTK_LIBRARIES} Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets OpenMP::OpenMP_CXX Eigen3::Eigen)

if(MINGW)
    target_link_options(ZweiCFD PRIVATE -static -static-libgcc -static-libstdc++)
endif()

add_custom_command(TARGET ZweiCFD POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/assets"
    "$<TARGET_FILE_DIR:ZweiCFD>/assets"
    VERBATIM
)

if(WIN32)
    # Copy all required DLL dependencies to the build output directory
    add_custom_command(TARGET ZweiCFD POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:ZweiCFD> $<TARGET_FILE_DIR:ZweiCFD>
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
endif()
