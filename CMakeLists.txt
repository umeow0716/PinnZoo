cmake_minimum_required(VERSION 3.10.1)

### ----- Generate dynamics for G7 OpenArm Quaternion Kinematics ----- ###
project(g7_openarm_quat) # Change this for each model
  
set(DIR models/g7_openarm/generated_code/g7_openarm_quat/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})