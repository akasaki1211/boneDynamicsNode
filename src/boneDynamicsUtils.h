#pragma once

// Disabled min/max macros in Windows API
#define NOMINMAX  

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>

namespace boneDynamicsUtils
{
    static constexpr MEulerRotation::RotationOrder kRotationOrder = MEulerRotation::RotationOrder::kXYZ;
    
    // Input data from attributes
    struct BoneInput
    {
        MVector boneTranslate;
        MVector boneJointOrient;
        MMatrix boneParentMatrix;
        MMatrix boneParentInverseMatrix;
        MVector boneScale;
        MVector boneInverseScale;

        MVector endTranslate;
        MVector endScale;

        MVector rotationOffset;

        double rootRadius;
        double radius;
    };

    // Calculated data such as matrices and positions
    struct PoseData
    {
        MEulerRotation rotationOffsetEuler; // used for reset and initialization
        MMatrix roMatrix;
        
        MVector boneWorldTranslate;
        MMatrix boneInitialWorldMatrixExcludeRO;
        MMatrix boneInitialWorldMatrix;
        MMatrix boneInitialParentInverseMatrix;
        
        MVector endWorldTranslate;
        MMatrix initialEndWorldMatrixExcludeRO; // used for disabled
        MMatrix initialEndWorldMatrix; // used for reset and initialization
        
        double rootRadius;
        double radius;
        double distance;
    };

    MMatrix resetMatrixScale(MMatrix originalMatrix);

    PoseData buildPoseData(const BoneInput& input);

    // used only in boneDynamivsVisualizer
    MMatrix buildSimulatedEndWorldMatrix(const BoneInput& input, const MVector& outputRotate);
}