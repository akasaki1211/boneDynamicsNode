#pragma once

// Disabled min/max macros in Windows API
#define NOMINMAX  

#include "boneDynamicsUtils.h"

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>

class boneDynamicsNode : public MPxNode
{
public:
    boneDynamicsNode();
    virtual ~boneDynamicsNode();
    
    static void* creator();
    virtual SchedulingType schedulingType() const;
    void getCacheSetup(const MEvaluationNode& evalNode, MNodeCacheDisablingInfo& disablingInfo, MNodeCacheSetupInfo& cacheSetupInfo, MObjectArray& monitoredAttributes) const override;

    static MStatus initialize();
    MStatus compute(const MPlug& plug, MDataBlock& data) override;

    static MTypeId s_id;

    static MObject s_enable;

    // time
    static MObject s_time;
    static MObject s_resetTime;
    static MObject s_fps;               // calculate deltatime 

    // transform
    static MObject s_offsetMatrix;      // offset current-position
    static MObject s_offsetMatrixWeight;

    static MObject s_boneTranslate;
    static MObject s_boneJointOrient;
    static MObject s_boneParentMatrix;
    static MObject s_boneParentInverseMatrix;
    static MObject s_endTranslate;

    static MObject s_rotationOffset;    // rotation value to add to initial pose

    // scale
    static MObject s_boneScale;         // use to calculate "End World Translate"
    static MObject s_boneInverseScale;  // use to calculate "End World Translate" and scale "Radius"
    static MObject s_endScale;          // use to scale "Radius"

    // dynamics
    static MObject s_damping;           // velocity damping
    static MObject s_elasticity;        // spring coefficient
    static MObject s_elasticForceFunction;
    static MObject s_stiffness;         // power to stay in place
    static MObject s_mass;
    static MObject s_gravity;           // gravity vector
    static MObject s_gravityMultiply;
    static MObject s_additionalForce;   // additional force
    static MObject s_additionalForceScale;

    // turbulence
    static MObject s_enableTurbulence;
    static MObject s_turbulenceSeed;
    static MObject s_turbulenceStrength;
    static MObject s_turbulenceVectorChangeScale;  // rate of change of the change-vector
    static MObject s_turbulenceVectorChangeMax;    // max value of the change-vector

    // angle limit
    static MObject s_enableAngleLimit;  // use angle limit
    static MObject s_angleLimit;        // angle limit

    // radius
    static MObject s_radius;            // radius

    // iterations
    static MObject s_iterations;        // iterations of constraints

    // collisions
    static MObject s_enableGroundCol;   // use ground colision
    static MObject s_groundHeight;      // ground height
    
    static MObject s_sphereCollider;    // sphereCollider array
    static MObject s_sphereColMtx;      // sphereCollider matrix
    static MObject s_sphereColRad;      // sphereCollider radius

    // (legacy) Using two matrices
    static MObject s_capsuleCollider;   // capsuleCollider array
    static MObject s_capsuleColMtxA;    // capsuleCollider matrix A
    static MObject s_capsuleColMtxB;    // capsuleCollider matrix B
    static MObject s_capsuleColRadA;    // capsuleCollider radius A
    static MObject s_capsuleColRadB;    // capsuleCollider radius B

    // Using one center matrix and height
    static MObject s_capsuleColliderInput;   // capsuleCollider array
    static MObject s_capsuleColliderMatrix;  // capsuleCollider matrix
    static MObject s_capsuleColliderHeight;  // capsuleCollider height
    static MObject s_capsuleColliderRadiusA; // capsuleCollider radius A
    static MObject s_capsuleColliderRadiusB; // capsuleCollider radius B

    static MObject s_iPlaneCollider;    // infinitePlaneCollider array
    static MObject s_iPlaneColMtx;      // infinitePlaneCollider matrix

    static MObject s_meshCollider;      // meshCollider array
    static MObject s_meshColCutoff;     // max distance for mesh collision detection
    
    // output
    static MObject s_outputRotate;       // output euler rotation

private:
    void angleLimit(const MVector& pivot, const MVector& a, MVector& b, const double limitAngle);
    void distanceConstraint(const MVector& pivot, MVector& point, double distance);
    void getClosestPoint(const MObject& mesh, const MPoint& position, MPoint& closestPoint, MVector& closestNormal);
    
    struct InitialPoseData : public boneDynamicsUtils::PoseData
    {
        // Inherits from boneDynamicsUtils::PoseData

        MMatrix offsetMatrix;
        double offsetMatrixWeight;
    };

    InitialPoseData buildInitialPoseData(MDataBlock& data) const;

    struct DynamicsParameters
    {
        double damping;
        double elasticity;
        short elasticForceFunction;
        double stiffness;
        double mass;
        MVector gravity;
        double gravityMultiply;
        
        MVector additionalForce;
        double additionalForceScale;
        bool enableTurbulence;
        int turbulenceSeed;
        double turbulenceStrength;
        double turbulenceVectorChangeScale;
        double turbulenceVectorChangeMax;
    };

    DynamicsParameters getDynamicsParameters(MDataBlock& data) const;
    
    // simulate state
    bool m_init;
    MMatrix m_prevOffsetMatrix;
    MVector m_position;
    MVector m_velocity;

    // turbulence state
    int m_lastSeed;
    int m_lastFrame;
    std::uint32_t m_rngState[4];
    MVector m_turbulenceVector;       // turbulence vector
    MVector m_turbulenceVectorChange; // vector that changes the turbulenceVector
};
