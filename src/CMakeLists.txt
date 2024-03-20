cmake_minimum_required(VERSION 3.10)

project(boneDynamicsNode)

include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake) 

set(SOURCE_FILES
   boneDynamicsNode.h
   boneDynamicsNode.cpp
   pluginMain.cpp
) 

set(LIBRARIES 
   OpenMaya 
   Foundation 
) 

build_plugin()