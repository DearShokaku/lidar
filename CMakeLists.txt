cmake_minimum_required(VERSION 3.16)
project(PointCloudViewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
find_package(OpenGL REQUIRED)

find_package(Eigen3 3.3 QUIET)
if(NOT Eigen3_FOUND)
    message(WARNING "Eigen3 not found. Using internal vector implementation.")
endif()

set(SOURCES
    src/main.cpp
    src/PointCloudViewer.cpp
    src/OpenGLWidget.cpp
    src/PointCloudData.cpp
    src/Octree.cpp
    src/PCDReader.cpp
    src/LASReader.cpp
    src/ColorMapper.cpp
)

set(HEADERS
    src/PointCloudViewer.h
    src/OpenGLWidget.h
    src/PointCloudData.h
    src/Octree.h
    src/PCDReader.h
    src/LASReader.h
    src/ColorMapper.h
)

add_executable(PointCloudViewer ${SOURCES} ${HEADERS})

target_link_libraries(PointCloudViewer PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    OpenGL::GL
    OpenGL::GLU
    $<$<BOOL:${Eigen3_FOUND}>:Eigen3::Eigen>
)

if(WIN32)
    set_target_properties(PointCloudViewer PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()
