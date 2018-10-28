cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)

project(mkz_localizer)

#set build mode;
#=======================================================================================================================
#set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#=======================================================================================================================

#boost
#=======================================================================================================================
find_package(Boost 1.60 COMPONENTS thread system filesystem timer chrono)
#=======================================================================================================================

#Qt
#=======================================================================================================================
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core)
find_package(Qt5Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
#=======================================================================================================================

#ROS
#=======================================================================================================================
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        tf
        )
#=======================================================================================================================

#OpenMp
#=================================================================
FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
#=================================================================

#cmake settings
#=======================================================================================================================
#set include files;
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/ ../include ../../include)

#set include files: *.h, *.hpp, *.hh;
file(GLOB_RECURSE PROJECT_HEADERS "include/*.h" "include/*.hpp" "include/*.hh" "../include/*.h" "../include/*.hpp" "../include/*.hh" "../../include/*.h" "../../include/*.hpp" "../../include/*.hh")

#set source files: *.cpp, *.c, *.cc;
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp" "src/*.c" "src/*.cc")

#set library files: *.a
file(GLOB_RECURSE PROJECT_LIBRARIES "lib/*.a")

#set library files path;
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/)

#set library output directory;
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)

#set executable output directory;
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
#=======================================================================================================================

#build excutable and library;
#=======================================================================================================================
if (${CMAKE_BUILD_TYPE} MATCHES "RELEASE")
    add_executable(app_mkz_localizer ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_MOC_UIS} ${PROJECT_MOC_UIS} ${PROJECT_MOC_QRCS})
    target_link_libraries(app_mkz_localizer ${PROJECT_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} proj glog gflags pthread)

    #add_library()

    #install built executable and library;
    install(TARGETS app_mkz_localizer
            RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/bin#executable;
            ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/lib#dynamic library, *.a;
            LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/lib#static lib, *.so;
            )
else ()
    add_executable(app_mkz_localizer_d ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_MOC_UIS} ${PROJECT_MOC_UIS} ${PROJECT_MOC_QRCS})
    target_link_libraries(app_mkz_localizer_d ${PROJECT_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} proj glog gflags pthread)

    #add_library()

    #install built executable and library;
    install(TARGETS app_mkz_localizer_d
            RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/bin#executable;
            ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/lib#dynamic library, *.a;
            LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../applications/lib#static lib, *.so;
            )
endif ()
