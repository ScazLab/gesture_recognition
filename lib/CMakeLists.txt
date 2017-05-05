# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(gesture_recognition_lib)

include_directories(include)

add_library(${PROJECT_NAME} include/${PROJECT_NAME}/gesture_recognition.h
                            src/${PROJECT_NAME}/gesture_recognition.cpp
)

add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} grt ${catkin_LIBRARIES} )
