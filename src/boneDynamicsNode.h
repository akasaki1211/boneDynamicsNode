#pragma once

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
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
    static MObject s_stiffness;         // power to stay in place
    static MObject s_mass;
    static MObject s_gravity;           // gravity vector
    static MObject s_gravityMultiply;

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
    
    // output
    static MObject s_outputRotate;       // output euler rotation

private:
    //static double getFPS();
    double degToRad(double deg);
    void angleLimit(const MVector& pivot, const MVector& a, MVector& b, const double limitAngle);
    void distanceConstraint(const MVector& pivot, MVector& point, double distance);

    static const MEulerRotation::RotationOrder ROTATION_ORDER = MEulerRotation::RotationOrder::kXYZ;
    
    bool m_init;
    MMatrix m_prevOffsetMatrix;
    MVector m_position;
    MVector m_velocity;
};