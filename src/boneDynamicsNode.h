#pragma once

// Disabled min/max macros in Windows API
#define NOMINMAX  

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MTime.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>

class boneDynamicsNode : public MPxNode
{
public:
    boneDynamicsNode();
    virtual ~boneDynamicsNode();
    
    static void* creator();
    virtual SchedulingType schedulingType() const;
    
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
    static MObject m_turbulenceVectorChangeScale;  // rate of change of the change-vector
    static MObject m_turbulenceVectorChangeMax;    // max value of the change-vector

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

    static MObject s_capsuleCollider;   // capsuleCollider array
    static MObject s_capsuleColMtxA;    // capsuleCollider matrix A
    static MObject s_capsuleColMtxB;    // capsuleCollider matrix B
    static MObject s_capsuleColRadA;    // capsuleCollider radius A
    static MObject s_capsuleColRadB;    // capsuleCollider radius B

    static MObject s_iPlaneCollider;    // infinitePlaneCollider array
    static MObject s_iPlaneColMtx;      // infinitePlaneCollider matrix

    static MObject s_meshCollider;      // meshCollider array
    static MObject s_meshColCutoff;     // max distance for mesh collision detection
    
    // output
    static MObject s_outputRotate;       // output euler rotation

private:
    //static double getFPS();
    double degToRad(double deg);
    void angleLimit(const MVector& pivot, const MVector& a, MVector& b, const double limitAngle);
    void distanceConstraint(const MVector& pivot, MVector& point, double distance);
    void getClosestPoint(const MObject& mesh, const MPoint& position, MPoint& closestPoint, MVector& closestNormal);
    
    inline uint32_t rotl(const uint32_t x, int k);
    double rand_double(const double scale);

    static const MEulerRotation::RotationOrder ROTATION_ORDER = MEulerRotation::RotationOrder::kXYZ;
    
    bool m_init;
    MMatrix m_prevOffsetMatrix;
    MVector m_position;
    MVector m_velocity;

    int m_lastSeed;
    int m_lastFrame;
    uint32_t m_rngState[4];
    MVector m_turbulenceVector;       // turbulence vector
    MVector m_turbulenceVectorChange; // vector that changes the turbulenceVector
};