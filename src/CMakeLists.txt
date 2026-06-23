cmake_minimum_required(VERSION 3.12)

project(boneDynamicsNode)

include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake) 

set(SOURCE_FILES
   boneDynamicsNode.h
   boneDynamicsNode.cpp
   boneDynamicsUtils.h
   boneDynamicsUtils.cpp
   mathUtils.h
   mathUtils.cpp
   nodeIds.h
   colliderNode/colliderGeometry.h
   colliderNode/colliderGeometry.cpp
   colliderNode/sphereColliderNode.h
   colliderNode/sphereColliderNode.cpp
   colliderNode/capsuleColliderNode.h
   colliderNode/capsuleColliderNode.cpp
   colliderNode/infinitePlaneColliderNode.h
   colliderNode/infinitePlaneColliderNode.cpp
   visualizerNode/visualizerGeometry.h
   visualizerNode/visualizerGeometry.cpp
   visualizerNode/boneDynamicsVisualizer.h
   visualizerNode/boneDynamicsVisualizer.cpp
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


# conformance settings
if(DEFINED CXX_STANDARD_VER AND NOT CXX_STANDARD_VER STREQUAL "")
   set_target_properties(${PROJECT_NAME} PROPERTIES
      CXX_STANDARD ${CXX_STANDARD_VER}
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS NO
   )
endif()

if(MSVC)
   target_compile_options(${PROJECT_NAME} PRIVATE
      /permissive-
      #/W4
      #/Zc:preprocessor
      /Zc:__cplusplus
      /utf-8
   )
endif()