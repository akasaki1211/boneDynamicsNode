cmake_minimum_required(VERSION 3.10)

project(boneDynamicsNode)

include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake) 

set(SOURCE_FILES
   boneDynamicsNode.h
   boneDynamicsNode.cpp
   mathUtils.h
   mathUtils.cpp
   nodeIds.h
   colliderNode/colliderGeometry.h
   colliderNode/colliderGeometry.cpp
   colliderNode/sphereColliderNode.h
   colliderNode/sphereColliderNode.cpp
   pluginMain.cpp
) 

set(LIBRARIES 
   OpenMaya 
   OpenMayaRender
   OpenMayaUI
   Foundation 
) 

find_opengl()

build_plugin()