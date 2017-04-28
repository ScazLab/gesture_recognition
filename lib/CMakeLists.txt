# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(gesture_recognition)

include_directories(${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/include)

add_library(${PROJECT_NAME} ${PROJECT_NAME}/include/gesture_recognition.h
                            ${PROJECT_NAME}/src/gesture_recognition.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
