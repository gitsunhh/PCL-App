cmake_minimum_required(VERSION 3.5)

set(Project_Name Point_Gui)
PROJECT(${Project_Name})

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Find the QtWidgets library
# find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)

#set(Qt5_DIR "D:\\programming\\Qt5.14.2\\5.14.2\\msvc2017_64\\lib\\cmake\\Qt5")
find_package(Qt5 COMPONENTS Widgets REQUIRED)

#find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(VTK_DIR "D:\\programming\\vs library\\VTK-8.2.0-build")
find_package(VTK REQUIRED)

set(project_SOURCES main.cpp PCL_App.cpp dlg_color_simple dlg_color_palette.cpp)

add_executable(${Project_Name}
    PCL_App.ui
    ${project_SOURCES}
    PCL_App.qrc
)

message ([main] "-------------------PCL STATUS------------------------------")
message ([main] "PCL_DIR = ${PCL_DIR}")
message ([main] "PCL_FOUND = ${PCL_FOUND}")
message ([main] "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message ([main] "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message ([main] "PCL_LIBRARIES = ${PCL_LIBRARIES}")
message ([main] "PCL_COMMON_LIBRARIES = ${PCL_COMMON_LIBRARIES}")
message ([main] "PCL_IO_LIBRARIES = ${PCL_IO_LIBRARIES}")
message ([main] "PCL_FILTERS_LIBRARIES = ${PCL_FILTERS_LIBRARIES}")
message ([main] "PCL_VISUALIZATION_LIBRARIES = ${PCL_VISUALIZATION_LIBRARIES}")
message ([main] "-------------------PCL STATUS------------------------------")

target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets)
#target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Qt5::Widgets)
