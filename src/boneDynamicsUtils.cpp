#include "boneDynamicsUtils.h"

#include <maya/MTransformationMatrix.h>

namespace boneDynamicsUtils
{
    MMatrix resetMatrixScale(MMatrix originalMatrix)
    {
        // override scale to 1 on the given matrix
        MTransformationMatrix transform(originalMatrix);
        double newScale[3] = {1.0, 1.0, 1.0};
        transform.setScale(newScale, MSpace::kTransform);
        return transform.asMatrix();
    }

    PoseData buildPoseData(const BoneInput& input)
    {
        PoseData pose;

        // rotation offset matrix
        pose.rotationOffsetEuler = MEulerRotation(input.rotationOffset, kRotationOrder);
        pose.roMatrix = pose.rotationOffsetEuler.asMatrix();

        // bone matrix
        MTransformationMatrix boneTransformationMatrix;
        boneTransformationMatrix.setTranslation(input.boneTranslate, MSpace::kWorld);
        const MMatrix boneMatrix = boneTransformationMatrix.asMatrix();

        const MMatrix joMatrix = MEulerRotation(input.boneJointOrient, kRotationOrder).asMatrix();
        
        // bone world position
        const MPoint boneWorldTranslatePoint = MPoint(input.boneTranslate) * input.boneParentMatrix;
        pose.boneWorldTranslate = MVector(boneWorldTranslatePoint);

        // bone initial world matrix
        pose.boneInitialWorldMatrixExcludeRO = joMatrix * boneMatrix * input.boneParentMatrix;
        pose.boneInitialWorldMatrix = pose.roMatrix * pose.boneInitialWorldMatrixExcludeRO;

        // bone initial world inverse matrix
        const MMatrix roInverseMatrix = !pose.roMatrix.isEquivalent(MMatrix::identity) ? pose.roMatrix.inverse() : MMatrix::identity;
        const MMatrix joInverseMatrix = !joMatrix.isEquivalent(MMatrix::identity) ? joMatrix.inverse() : MMatrix::identity;
        pose.boneInitialParentInverseMatrix = input.boneParentInverseMatrix * joInverseMatrix * roInverseMatrix;

        // end translate with scale
        const MVector boneInverseScale(
            1.0 / input.boneInverseScale.x, 
            1.0 / input.boneInverseScale.y, 
            1.0 / input.boneInverseScale.z
        );
        const MPoint scaledEndTranslate(
            input.endTranslate.x * input.boneScale.x * boneInverseScale.x, 
            input.endTranslate.y * input.boneScale.y * boneInverseScale.y, 
            input.endTranslate.z * input.boneScale.z * boneInverseScale.z
        );

        MTransformationMatrix endTransformationMatrix;
        endTransformationMatrix.setTranslation(scaledEndTranslate, MSpace::kWorld);

        // end world position
        const MPoint endWorldTranslatePoint = scaledEndTranslate * pose.boneInitialWorldMatrix;
        pose.endWorldTranslate = MVector(endWorldTranslatePoint);

        // end init world matrix        
        pose.initialEndWorldMatrix = resetMatrixScale(endTransformationMatrix.asMatrix() * pose.boneInitialWorldMatrix);
        pose.initialEndWorldMatrixExcludeRO = resetMatrixScale(endTransformationMatrix.asMatrix() * pose.boneInitialWorldMatrixExcludeRO);

        // radius
        const MTransformationMatrix boneParentTransformationMatrix(input.boneParentMatrix);
        double boneParentScale[3];
        boneParentTransformationMatrix.getScale(boneParentScale, MSpace::kWorld);
        pose.radius = input.radius * input.endScale.z * boneInverseScale.z * boneParentScale[2]; // Specified by scale Z
        pose.rootRadius = input.rootRadius * input.boneScale.z * boneInverseScale.z * boneParentScale[2]; // Specified by scale Z

        // bone length
        pose.distance = (pose.endWorldTranslate - pose.boneWorldTranslate).length();
        
        return pose;
    }

    MMatrix buildSimulatedEndWorldMatrix(const BoneInput& input, const MVector& outputRotate)
    {
        // bone matrix
        MTransformationMatrix boneTransformationMatrix;
        boneTransformationMatrix.setTranslation(input.boneTranslate, MSpace::kWorld);
        const MMatrix boneMatrix = boneTransformationMatrix.asMatrix();

        // bone joint orient matrix
        const MMatrix joMatrix = MEulerRotation(input.boneJointOrient, kRotationOrder).asMatrix();

        // bone rotate matrix
        const MMatrix boneRotateMatrix = MEulerRotation(outputRotate, kRotationOrder).asMatrix();
        
        // simulated bone world matrix
        const MMatrix simulatedBoneWorldMatrix = boneRotateMatrix * joMatrix * boneMatrix * input.boneParentMatrix;

        // end translate with scale
        const MVector boneInverseScale(
            1.0 / input.boneInverseScale.x,
            1.0 / input.boneInverseScale.y,
            1.0 / input.boneInverseScale.z
        );

        const MPoint scaledEndTranslate(
            input.endTranslate.x * input.boneScale.x * boneInverseScale.x,
            input.endTranslate.y * input.boneScale.y * boneInverseScale.y,
            input.endTranslate.z * input.boneScale.z * boneInverseScale.z
        );

        MTransformationMatrix endTransformationMatrix;
        endTransformationMatrix.setTranslation(scaledEndTranslate, MSpace::kWorld);

        return resetMatrixScale(endTransformationMatrix.asMatrix() * simulatedBoneWorldMatrix);
    }
}