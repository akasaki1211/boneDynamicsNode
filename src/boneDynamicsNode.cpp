#include "boneDynamicsNode.h"
#include "boneDynamicsUtils.h"
#include "nodeIds.h"
#include "mathUtils.h"

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MTime.h>
#include <maya/MPoint.h>
#include <maya/MTransformationMatrix.h>

#include <algorithm> // std::max, std::min

namespace
{
    /*double getFPS()
    {
        float fps = 24.0f;
        MTime::Unit unit = MTime::uiUnit();
        if (unit != MTime::kInvalid)
        {
            MTime time(1.0, MTime::kSeconds);
            fps = static_cast<float>(time.as(unit));
        }
        if (fps <= 0.f)
        {
            fps = 24.0f;
        }
        return fps;
    }*/
}

MTypeId boneDynamicsNode::s_id(nodeIds::boneDynamicsNode);

MObject boneDynamicsNode::s_enable;

MObject boneDynamicsNode::s_time;
MObject boneDynamicsNode::s_resetTime;
MObject boneDynamicsNode::s_fps;

MObject boneDynamicsNode::s_offsetMatrix;
MObject boneDynamicsNode::s_offsetMatrixWeight;

MObject boneDynamicsNode::s_boneTranslate;
MObject boneDynamicsNode::s_boneJointOrient;
MObject boneDynamicsNode::s_boneParentMatrix;
MObject boneDynamicsNode::s_boneParentInverseMatrix;
MObject boneDynamicsNode::s_endTranslate;

MObject boneDynamicsNode::s_rotationOffset;

MObject boneDynamicsNode::s_boneScale;
MObject boneDynamicsNode::s_boneInverseScale;
MObject boneDynamicsNode::s_endScale;

MObject boneDynamicsNode::s_damping;
MObject boneDynamicsNode::s_elasticity;
MObject boneDynamicsNode::s_elasticForceFunction;
MObject boneDynamicsNode::s_stiffness;
MObject boneDynamicsNode::s_mass;
MObject boneDynamicsNode::s_gravityMultiply;
MObject boneDynamicsNode::s_gravity;
MObject boneDynamicsNode::s_additionalForce;
MObject boneDynamicsNode::s_additionalForceScale;

MObject boneDynamicsNode::s_enableTurbulence;
MObject boneDynamicsNode::s_turbulenceSeed;
MObject boneDynamicsNode::s_turbulenceStrength;
MObject boneDynamicsNode::s_turbulenceVectorChangeScale;
MObject boneDynamicsNode::s_turbulenceVectorChangeMax;

MObject boneDynamicsNode::s_enableAngleLimit;
MObject boneDynamicsNode::s_angleLimit;

MObject boneDynamicsNode::s_radius;

MObject boneDynamicsNode::s_iterations;

MObject boneDynamicsNode::s_enableGroundCol;
MObject boneDynamicsNode::s_groundHeight;

MObject boneDynamicsNode::s_sphereCollider; 
MObject boneDynamicsNode::s_sphereColMtx;
MObject boneDynamicsNode::s_sphereColRad;

MObject boneDynamicsNode::s_capsuleCollider;
MObject boneDynamicsNode::s_capsuleColMtxA;
MObject boneDynamicsNode::s_capsuleColMtxB;
MObject boneDynamicsNode::s_capsuleColRadA;
MObject boneDynamicsNode::s_capsuleColRadB;

MObject boneDynamicsNode::s_capsuleColliderInput;
MObject boneDynamicsNode::s_capsuleColliderMatrix;
MObject boneDynamicsNode::s_capsuleColliderHeight;
MObject boneDynamicsNode::s_capsuleColliderRadiusA;
MObject boneDynamicsNode::s_capsuleColliderRadiusB;

MObject boneDynamicsNode::s_iPlaneCollider;
MObject boneDynamicsNode::s_iPlaneColMtx;

MObject boneDynamicsNode::s_meshCollider;
MObject boneDynamicsNode::s_meshColCutoff;

MObject boneDynamicsNode::s_outputRotate;

boneDynamicsNode::boneDynamicsNode() : m_init(true), m_lastSeed(-1), m_lastFrame(-1) {
    m_rngState[0] = 0;
    m_rngState[1] = 0;
    m_rngState[2] = 0;
    m_rngState[3] = 0;
}

boneDynamicsNode::~boneDynamicsNode(){}

void* boneDynamicsNode::creator()
{
    return new boneDynamicsNode;
}

boneDynamicsNode::SchedulingType boneDynamicsNode::schedulingType() const
{
    return kParallel;
}

void boneDynamicsNode::getCacheSetup(const MEvaluationNode& evalNode, MNodeCacheDisablingInfo& disablingInfo, MNodeCacheSetupInfo& cacheSetupInfo, MObjectArray& monitoredAttributes) const
{
    bool enabled = false;
    MStatus status;
    MObject nodeObj = thisMObject();
    MPlug enablePlug(nodeObj, s_enable);
    if (!enablePlug.isNull()) {
        status = enablePlug.getValue(enabled);
    }

    // disable cached playback
    if (enabled) {
        disablingInfo.setCacheDisabled(true);
        disablingInfo.setReason("boneDynamicsNode does not support cached playback.");
    }

    monitoredAttributes.append(s_enable);
}

MStatus boneDynamicsNode::initialize()
{
    MFnNumericAttribute nAttr;
    MFnCompoundAttribute cmpAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;
    MFnTypedAttribute tAttr;
    MFnEnumAttribute eAttr;
    MObject x, y, z;

    s_enable = nAttr.create("enable", "en", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);

    // time attributes
    s_time = uAttr.create("time", "t", MFnUnitAttribute::kTime, 0.0);
    uAttr.setKeyable(false);

    s_resetTime = uAttr.create("resetTime", "rt", MFnUnitAttribute::kTime, 0.0);
    uAttr.setKeyable(true);

    s_fps = nAttr.create("fps", "fps", MFnNumericData::kDouble, 30.0);
    nAttr.setKeyable(true);
    nAttr.setMin(1);

    // input attributes
    s_offsetMatrix = mAttr.create("offsetMatrix", "ofmtx");
    mAttr.setKeyable(true);

    s_offsetMatrixWeight = nAttr.create("offsetMatrixWeight", "ofmtxw", MFnNumericData::kDouble, 1.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(1);

    x = nAttr.create("boneTranslateX", "btx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("boneTranslateY", "bty", MFnNumericData::kDouble, 0.0);
    z = nAttr.create("boneTranslateZ", "btz", MFnNumericData::kDouble, 0.0);
    s_boneTranslate = nAttr.create("boneTranslate", "bt", x, y, z);
    nAttr.setKeyable(true);

    x = uAttr.create("boneJointOrientX", "bjox", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("boneJointOrientY", "bjoy", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("boneJointOrientZ", "bjoz", MFnUnitAttribute::kAngle, 0.0);
    s_boneJointOrient = nAttr.create("boneJointOrient", "bjo", x, y, z);
    nAttr.setKeyable(true);

    s_boneParentMatrix = mAttr.create("boneParentMatrix", "bpmtx");
    mAttr.setKeyable(true);

    s_boneParentInverseMatrix = mAttr.create("boneParentInverseMatrix", "bpimtx");
    mAttr.setKeyable(true);

    x = nAttr.create("endTranslateX", "etx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("endTranslateY", "ety", MFnNumericData::kDouble, 0.0);
    z = nAttr.create("endTranslateZ", "etz", MFnNumericData::kDouble, 0.0);
    s_endTranslate = nAttr.create("endTranslate", "et", x, y, z);
    nAttr.setKeyable(true);

    x = uAttr.create("rotationOffsetX", "rox", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("rotationOffsetY", "roy", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("rotationOffsetZ", "roz", MFnUnitAttribute::kAngle, 0.0);
    s_rotationOffset = nAttr.create("rotationOffset", "ro", x, y, z);
    nAttr.setKeyable(true);

    x = nAttr.create("boneScaleX", "bsx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("boneScaleY", "bsy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("boneScaleZ", "bsz", MFnNumericData::kDouble, 1.0);
    s_boneScale = nAttr.create("boneScale", "bs", x, y, z);
    nAttr.setKeyable(true);

    x = nAttr.create("boneInverseScaleX", "bisx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("boneInverseScaleY", "bisy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("boneInverseScaleZ", "bisz", MFnNumericData::kDouble, 1.0);
    s_boneInverseScale = nAttr.create("boneInverseScale", "bis", x, y, z);
    nAttr.setKeyable(true);

    x = nAttr.create("endScaleX", "esx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("endScaleY", "esy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("endScaleZ", "esz", MFnNumericData::kDouble, 1.0);
    s_endScale = nAttr.create("endScale", "es", x, y, z);
    nAttr.setKeyable(true);

    // dynamics attributes
    s_damping = nAttr.create("damping", "damp", MFnNumericData::kDouble, 0.1);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(1);

    s_elasticity = nAttr.create("elasticity", "elas", MFnNumericData::kDouble, 30.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_elasticForceFunction = eAttr.create("elasticForceFunction", "eff", 0);
    eAttr.addField("Linear", 0);
    eAttr.addField("Quadratic", 1);
    eAttr.addField("Cubic", 2);
    eAttr.setKeyable(true);

    s_stiffness = nAttr.create("stiffness", "stif", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(1);

    s_mass = nAttr.create("mass", "m", MFnNumericData::kDouble, 1.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0.001);

    x = nAttr.create("gravityX", "gx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("gravityY", "gy", MFnNumericData::kDouble, -980.0);
    z = nAttr.create("gravityZ", "gz", MFnNumericData::kDouble, 0.0);
    s_gravity = nAttr.create("gravity", "g", x, y, z);
    nAttr.setKeyable(true);

    s_gravityMultiply = nAttr.create("gravityMultiply", "gm", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(1);

    x = nAttr.create("additionalForceX", "afx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("additionalForceY", "afy", MFnNumericData::kDouble, 0.0);
    z = nAttr.create("additionalForceZ", "afz", MFnNumericData::kDouble, 0.0);
    s_additionalForce = nAttr.create("additionalForce", "af", x, y, z);
    nAttr.setKeyable(true);

    s_additionalForceScale = nAttr.create("additionalForceScale", "afs", MFnNumericData::kDouble, 1.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    // turbulence
    s_enableTurbulence = nAttr.create("enableTurbulence", "enw", MFnNumericData::kBoolean, false);
    nAttr.setKeyable(true);

    s_turbulenceSeed = nAttr.create("turbulenceSeed", "wsd", MFnNumericData::kInt, 0);
    nAttr.setMin(0);
    nAttr.setKeyable(true);

    s_turbulenceStrength = nAttr.create("turbulenceStrength", "wst", MFnNumericData::kDouble, 10.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_turbulenceVectorChangeScale = nAttr.create("turbulenceVectorChangeScale", "wvcs", MFnNumericData::kDouble, 0.05);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_turbulenceVectorChangeMax = nAttr.create("turbulenceVectorChangeMax", "wvcm", MFnNumericData::kDouble, 0.1);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    // angle limit
    s_enableAngleLimit = nAttr.create("enableAngleLimit", "eal", MFnNumericData::kBoolean, false);
    nAttr.setKeyable(true);

    s_angleLimit = nAttr.create("angleLimit", "al", MFnNumericData::kDouble, 60.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(360);

    // radius
    s_radius = nAttr.create("radius", "r", MFnNumericData::kDouble, 0.0); // TODO: Change to MFnUnitAttribute::kDistance
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    // constraint attributes
    s_iterations = nAttr.create("iterations", "iter", MFnNumericData::kLong, 5);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(10);

    // collider attributes
    // groundCollider
    s_enableGroundCol = nAttr.create("enableGroundCol", "gc", MFnNumericData::kBoolean, false);
    nAttr.setKeyable(true);

    s_groundHeight = nAttr.create("groundHeight", "gh", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);

    // sphereCollider
    s_sphereColMtx = mAttr.create("sphereColMatrix", "scmtx");
    mAttr.setKeyable(true);
    
    s_sphereColRad = nAttr.create("sphereColRadius", "scrad", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_sphereCollider = cmpAttr.create("sphereCollider", "sc");
    cmpAttr.setArray(true);
    cmpAttr.addChild(s_sphereColMtx);
    cmpAttr.addChild(s_sphereColRad);
    cmpAttr.setReadable(true);
    cmpAttr.setUsesArrayDataBuilder(true);

    // capsuleCollider
    s_capsuleColMtxA = mAttr.create("capsuleColMatrixA", "ccmtxa");
    mAttr.setKeyable(true);
    
    s_capsuleColMtxB = mAttr.create("capsuleColMatrixB", "ccmtxb");
    mAttr.setKeyable(true);

    s_capsuleColRadA = nAttr.create("capsuleColRadiusA", "ccrada", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_capsuleColRadB = nAttr.create("capsuleColRadiusB", "ccradb", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_capsuleCollider = cmpAttr.create("capsuleCollider", "cc");
    cmpAttr.setArray(true);
    cmpAttr.addChild(s_capsuleColMtxA);
    cmpAttr.addChild(s_capsuleColMtxB);
    cmpAttr.addChild(s_capsuleColRadA);
    cmpAttr.addChild(s_capsuleColRadB);
    cmpAttr.setReadable(true);
    cmpAttr.setUsesArrayDataBuilder(true);

    // capsuleCollider (*Using one center matrix and height)
    s_capsuleColliderMatrix = mAttr.create("capsuleColliderMatrix", "ccimtx");
    mAttr.setKeyable(true);

    s_capsuleColliderHeight = nAttr.create("capsuleColliderHeight", "ccih", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_capsuleColliderRadiusA = nAttr.create("capsuleColliderRadiusA", "ccirada", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_capsuleColliderRadiusB = nAttr.create("capsuleColliderRadiusB", "cciradb", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    s_capsuleColliderInput = cmpAttr.create("capsuleColliderInput", "cci");
    cmpAttr.setArray(true);
    cmpAttr.addChild(s_capsuleColliderMatrix);
    cmpAttr.addChild(s_capsuleColliderHeight);
    cmpAttr.addChild(s_capsuleColliderRadiusA);
    cmpAttr.addChild(s_capsuleColliderRadiusB);
    cmpAttr.setReadable(true);
    cmpAttr.setUsesArrayDataBuilder(true);

    // infinitePlaneCollider
    s_iPlaneColMtx = mAttr.create("infinitePlaneColMatrix", "pcmtx");
    mAttr.setKeyable(true);

    s_iPlaneCollider = cmpAttr.create("infinitePlaneCollider", "pc");
    cmpAttr.setArray(true);
    cmpAttr.addChild(s_iPlaneColMtx);
    cmpAttr.setReadable(true);
    cmpAttr.setUsesArrayDataBuilder(true);

    // meshCollider
    s_meshCollider = tAttr.create("meshCollider", "mc", MFnData::kMesh);
    tAttr.setArray(true);
    tAttr.setReadable(true);
    tAttr.setUsesArrayDataBuilder(true);

    s_meshColCutoff = nAttr.create("meshColCutoff", "mcc", MFnNumericData::kDouble, 10.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    // output attributes
    x = uAttr.create("outputRotateX", "outrx", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("outputRotateY", "outry", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("outputRotateZ", "outrz", MFnUnitAttribute::kAngle, 0.0);
    s_outputRotate = nAttr.create("outputRotate", "outr", x, y, z);
    nAttr.setStorable(false);
    nAttr.setWritable(false);

    // addAttribute
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_enable));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_time));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_resetTime));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_fps));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_offsetMatrix));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_offsetMatrixWeight));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneTranslate));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneJointOrient));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneParentMatrix));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneParentInverseMatrix));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneScale));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_boneInverseScale));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_endTranslate));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_endScale));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_rotationOffset));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_damping));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_elasticity));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_elasticForceFunction));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_stiffness));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_mass));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_gravity));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_gravityMultiply));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_additionalForce));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_additionalForceScale));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_enableTurbulence));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_turbulenceSeed));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_turbulenceStrength));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_turbulenceVectorChangeScale));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_turbulenceVectorChangeMax));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_enableAngleLimit));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_angleLimit));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_radius));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_iterations));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_enableGroundCol));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_groundHeight));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_sphereCollider));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_capsuleCollider));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_capsuleColliderInput));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_iPlaneCollider));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_meshCollider));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_meshColCutoff));

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_outputRotate));
    
    // attributeAffects
    const MObject simulationInputs[] = {
        s_enable,

        s_time,
        s_resetTime,
        s_fps,

        s_offsetMatrix,
        s_offsetMatrixWeight,

        s_boneTranslate,
        s_boneJointOrient,
        s_boneParentMatrix,
        s_boneParentInverseMatrix,
        s_endTranslate,

        s_rotationOffset,

        s_boneScale,
        s_boneInverseScale,
        s_endScale,

        s_damping,
        s_elasticity,
        s_elasticForceFunction,
        s_stiffness,
        s_mass,
        s_gravity,
        s_gravityMultiply,
        s_additionalForce,
        s_additionalForceScale,

        s_enableTurbulence,
        s_turbulenceSeed,
        s_turbulenceStrength,
        s_turbulenceVectorChangeScale,
        s_turbulenceVectorChangeMax,

        s_enableAngleLimit,
        s_angleLimit,

        s_radius,

        s_iterations,

        s_enableGroundCol,
        s_groundHeight,
        s_sphereColMtx,
        s_sphereColRad,
        s_sphereCollider,
        s_capsuleColMtxA,
        s_capsuleColMtxB,
        s_capsuleColRadA,
        s_capsuleColRadB,
        s_capsuleCollider,
        s_capsuleColliderMatrix,
        s_capsuleColliderHeight,
        s_capsuleColliderRadiusA,
        s_capsuleColliderRadiusB,
        s_capsuleColliderInput,
        s_iPlaneColMtx,
        s_iPlaneCollider,
        s_meshCollider,
        s_meshColCutoff
    };

    for (const MObject& input : simulationInputs)
    {
        CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(input, s_outputRotate));
    }

    return MS::kSuccess;
}

void boneDynamicsNode::angleLimit(const MVector& pivot, const MVector& a, MVector& b, const double limitAngle)
{
    const MVector initVec = a - pivot;
    const MVector currentVec = b - pivot;
    const MQuaternion quat(initVec, currentVec);

    MVector axis;
    double currentAngle;
    quat.getAxisAngle(axis, currentAngle);

    if (currentAngle > limitAngle)
    {
        const double rotateAngle = limitAngle - currentAngle;
        const MVector axisNormal = axis.normal();

        const MVector rotatedVec = currentVec * cos(rotateAngle) + (axisNormal ^ currentVec) * sin(rotateAngle);

        b = pivot + rotatedVec;
    }
}

void boneDynamicsNode::distanceConstraint(const MVector& pivot, MVector& point, double distance)
{
    // update point
    point = pivot + ((point - pivot).normal() * distance);
}

void boneDynamicsNode::getClosestPoint(const MObject& mesh, const MPoint& point, MPoint& closestPoint, MVector& closestNormal)
{
    MFnMesh fnMesh(mesh);
    fnMesh.getClosestPointAndNormal(point, closestPoint, closestNormal, MSpace::kWorld);
}

boneDynamicsNode::InitialPoseData boneDynamicsNode::buildInitialPoseData(MDataBlock& data) const
{
    InitialPoseData initialPose;

    // offset matrix inputs
    initialPose.offsetMatrix = data.inputValue(s_offsetMatrix).asMatrix();
    initialPose.offsetMatrixWeight = data.inputValue(s_offsetMatrixWeight).asDouble();

    boneDynamicsUtils::BoneInput input;

    // bone input
    input.boneTranslate = data.inputValue(s_boneTranslate).asVector();
    input.boneJointOrient = data.inputValue(s_boneJointOrient).asVector();
    input.boneParentMatrix = data.inputValue(s_boneParentMatrix).asMatrix();
    input.boneParentInverseMatrix = data.inputValue(s_boneParentInverseMatrix).asMatrix();
    input.boneScale = data.inputValue(s_boneScale).asVector();
    input.boneInverseScale = data.inputValue(s_boneInverseScale).asVector();
    
    // end input
    input.endTranslate = data.inputValue(s_endTranslate).asVector();
    input.endScale = data.inputValue(s_endScale).asVector();

    // rotation offset input
    input.rotationOffset = data.inputValue(s_rotationOffset).asVector();

    // radius input
    input.radius = data.inputValue(s_radius).asDouble();

    // build pose data
    const boneDynamicsUtils::PoseData poseData = boneDynamicsUtils::buildPoseData(input);

    // cast to base PoseData
    static_cast<boneDynamicsUtils::PoseData&>(initialPose) = poseData;

    return initialPose;
}

boneDynamicsNode::DynamicsParameters boneDynamicsNode::getDynamicsParameters(MDataBlock& data) const
{
    DynamicsParameters params;

    params.damping = data.inputValue(s_damping).asDouble();
    params.elasticity = data.inputValue(s_elasticity).asDouble();
    params.elasticForceFunction = data.inputValue(s_elasticForceFunction).asShort();
    params.stiffness = data.inputValue(s_stiffness).asDouble();
    params.mass = data.inputValue(s_mass).asDouble();
    params.gravity = data.inputValue(s_gravity).asVector();
    params.gravityMultiply = data.inputValue(s_gravityMultiply).asDouble();
    params.additionalForce = data.inputValue(s_additionalForce).asVector();
    params.additionalForceScale = data.inputValue(s_additionalForceScale).asDouble();
    
    params.enableTurbulence = data.inputValue(s_enableTurbulence).asBool();
    params.turbulenceSeed = data.inputValue(s_turbulenceSeed).asInt();
    params.turbulenceStrength = data.inputValue(s_turbulenceStrength).asDouble();
    params.turbulenceVectorChangeScale = data.inputValue(s_turbulenceVectorChangeScale).asDouble();
    params.turbulenceVectorChangeMax = data.inputValue(s_turbulenceVectorChangeMax).asDouble();

    return params;
}

MStatus boneDynamicsNode::compute(const MPlug& plug, MDataBlock& data)
{
    const MPlug computePlug = plug.isChild() ? plug.parent() : plug;
    
    if (computePlug != s_outputRotate)
    {
        return MS::kUnknownParameter;
    }

    // output data handles
    MDataHandle& outputRotateHandle = data.outputValue(s_outputRotate);
    
    // build initial pose data
    InitialPoseData initialPose = buildInitialPoseData(data);
    
    // check enable
    const bool enable = data.inputValue(s_enable).asBool();
    if (!enable)
    {
        outputRotateHandle.set3Double(0.0, 0.0, 0.0);
        outputRotateHandle.setClean();

        return MS::kSuccess;
    }

    // dynamic parameters
    DynamicsParameters dynamicsParams = getDynamicsParameters(data);

    // time
    const MTime& time = data.inputValue(s_time).asTime();
    const MTime& resetTime = data.inputValue(s_resetTime).asTime();
    
    // reset and initialization
    if (time <= resetTime || m_init) {
        m_prevOffsetMatrix = initialPose.offsetMatrix;
        m_position = initialPose.endWorldTranslate;
        m_velocity = MVector();
        m_init = false;

        if (dynamicsParams.enableTurbulence) {
            m_turbulenceVector = MVector();
            m_turbulenceVectorChange = MVector();
            m_lastSeed = -1;
            m_lastFrame = -1;
            m_rngState[0] = 0;
            m_rngState[1] = 0;
            m_rngState[2] = 0;
            m_rngState[3] = 0;
        }

        outputRotateHandle.set3Double(
            initialPose.rotationOffsetEuler.x, 
            initialPose.rotationOffsetEuler.y, 
            initialPose.rotationOffsetEuler.z
        );
        outputRotateHandle.setClean();
        
        return MS::kSuccess;
    }

    // delta time
    //double dt = 1.0 / getFPS();
    const double fps = data.inputValue(s_fps).asDouble();
    const double dt = 1.0 / fps;
    const double dt2 = dt * dt;

    // position offset
    const MPoint offsetedPosition = MPoint(m_position) * m_prevOffsetMatrix.inverse() * initialPose.offsetMatrix;
    m_position = MVector(offsetedPosition) * initialPose.offsetMatrixWeight + m_position * (1.0 - initialPose.offsetMatrixWeight);
    
    // velocity damping
    m_velocity *= (1.0 - dynamicsParams.damping);
    
    // add velocity
    MVector nextPosition = m_position + (m_velocity * dt);

    // turbulence vector
    if (dynamicsParams.enableTurbulence) {
        int frame = static_cast<int>(time.as(MTime::uiUnit()));

        if (dynamicsParams.turbulenceSeed != m_lastSeed || frame != m_lastFrame) {

            // Init xoshiro128++
            std::uint32_t s = static_cast<std::uint32_t>(dynamicsParams.turbulenceSeed) ^ (static_cast<std::uint32_t>(frame) * 0x9e3779b9u);
            for (int i = 0; i < 4; ++i) {
                s = s * 1664525u + 1013904223u;
                m_rngState[i] = s;
            }
            m_lastSeed = dynamicsParams.turbulenceSeed;
            m_lastFrame = frame;
        }
        
        m_turbulenceVectorChange += MVector(
            mathUtils::randomDouble(m_rngState, dynamicsParams.turbulenceVectorChangeScale),
            mathUtils::randomDouble(m_rngState, dynamicsParams.turbulenceVectorChangeScale),
            mathUtils::randomDouble(m_rngState, dynamicsParams.turbulenceVectorChangeScale)
        );

        m_turbulenceVectorChange.x = std::max(-dynamicsParams.turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.x, dynamicsParams.turbulenceVectorChangeMax));
        m_turbulenceVectorChange.y = std::max(-dynamicsParams.turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.y, dynamicsParams.turbulenceVectorChangeMax));
        m_turbulenceVectorChange.z = std::max(-dynamicsParams.turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.z, dynamicsParams.turbulenceVectorChangeMax));

        m_turbulenceVector += m_turbulenceVectorChange;
        m_turbulenceVector.normalize();
    }

    // external force
    // gravity
    MVector externalForce = dynamicsParams.gravity * dynamicsParams.gravityMultiply;

    // spring force
    const MVector springVector = (initialPose.endWorldTranslate - nextPosition);
    const double springVectorLength = springVector.length();
    MVector springForce;

    switch (dynamicsParams.elasticForceFunction) // experimental
    {
        case 0: // Linear
            springForce = (springVector * dynamicsParams.elasticity) / dynamicsParams.mass;
            break;
        case 1: // Quadratic
            springForce = (springVector * ((springVectorLength + 1.0) * dynamicsParams.elasticity)) / dynamicsParams.mass;
            break;
        case 2: // Cubic
            springForce = (springVector * ((springVectorLength + 1.0) * (springVectorLength + 1.0) * dynamicsParams.elasticity)) / dynamicsParams.mass;
            break;
        default:
            break;
    }
    
    // clamp spring force
    const double maxSpringForce = springVectorLength / dt2;
    if (springForce.length() > maxSpringForce)
    {
        springForce = springForce.normal() * maxSpringForce;
    }

    externalForce += springForce;
    
    // additional force
    externalForce += (dynamicsParams.additionalForce * dynamicsParams.additionalForceScale) / dynamicsParams.mass;
    
    // turbulence force
    if (dynamicsParams.enableTurbulence) {
        externalForce += (m_turbulenceVector * dynamicsParams.turbulenceStrength) / dynamicsParams.mass;
    }

    // apply external force
    nextPosition += externalForce * dt2;
    
    // apply stiffness
    nextPosition += (m_position - nextPosition) * dynamicsParams.stiffness;

    // angle limit
    const bool enableAngleLimit = data.inputValue(s_enableAngleLimit).asBool();
    const double angleLimitDegrees = data.inputValue(s_angleLimit).asDouble();

    // collision
    const bool enableGroundCol = data.inputValue(s_enableGroundCol).asBool();
    const double groundHeight = data.inputValue(s_groundHeight).asDouble();

    MArrayDataHandle& sphereColArrayHandle = data.inputArrayValue(s_sphereCollider);
    const unsigned int scCount = sphereColArrayHandle.elementCount();
        
    MArrayDataHandle& capsuleColArrayHandle = data.inputArrayValue(s_capsuleCollider);
    const unsigned int ccCount = capsuleColArrayHandle.elementCount();

    MArrayDataHandle& capsuleColiderInputArrayHandle = data.inputArrayValue(s_capsuleColliderInput);
    const unsigned int cciCount = capsuleColiderInputArrayHandle.elementCount();
        
    MArrayDataHandle& iPlaneColArrayHandle = data.inputArrayValue(s_iPlaneCollider);
    const unsigned int pcCount = iPlaneColArrayHandle.elementCount();

    MArrayDataHandle& meshColArrayHandle = data.inputArrayValue(s_meshCollider);
    const unsigned int mcCount = meshColArrayHandle.elementCount();
    const double meshColCutoff = data.inputValue(s_meshColCutoff).asDouble();
    
    MTransformationMatrix sphereCol_m;
    MVector sphereCol_p;
    double sphereCol_s[3];
    double sphereCol_r;

    MTransformationMatrix capsuleCol_mA;
    MTransformationMatrix capsuleCol_mB;
    MVector capsuleCol_pA;
    MVector capsuleCol_pB;
    double capsuleCol_s[3];
    double capsuleCol_rA;
    double capsuleCol_rB;
    
    // Using one center matrix and height
    MMatrix capsuleColMatrix;
    double capsuleColHeight;
    MVector capsuleColPointA;
    MVector capsuleColPointB;
    double capsuleColScale[3];
    double capsuleColRadiusA;
    double capsuleColRadiusB;
    
    MTransformationMatrix iPlaneCol_m;
    MVector iPlaneCol_p;
    MVector iPlaneCol_n;
    MObject meshCol;

    MVector v;
    double r;

    const long iter = data.inputValue(s_iterations).asLong();

    for (int i = 0; i < iter; i++)
    {
        // distance constraint
        distanceConstraint(initialPose.boneWorldTranslate, nextPosition, initialPose.distance);
        
        //sphere collision
        for (unsigned int i = 0; i < scCount; i++) {
            sphereColArrayHandle.jumpToArrayElement(i);
            MDataHandle& sphereCollider = sphereColArrayHandle.inputValue();
            sphereCol_m = sphereCollider.child(s_sphereColMtx).asMatrix();
            sphereCol_p = sphereCol_m.getTranslation(MSpace::kWorld);
            sphereCol_m.getScale(sphereCol_s, MSpace::kWorld);
            sphereCol_r = sphereCollider.child(s_sphereColRad).asDouble() * sphereCol_s[2];
                
            v = nextPosition - sphereCol_p;
            r = sphereCol_r + initialPose.radius;
            if (v * v < r * r)
            {
                nextPosition = sphereCol_p + (v.normal() * r);
            };
        };

        //capsule collision
        for (unsigned int i = 0; i < ccCount; i++) {
            capsuleColArrayHandle.jumpToArrayElement(i);
            MDataHandle& capsuleCollider = capsuleColArrayHandle.inputValue();
            capsuleCol_mA = capsuleCollider.child(s_capsuleColMtxA).asMatrix();
            capsuleCol_pA = capsuleCol_mA.getTranslation(MSpace::kWorld);
            capsuleCol_mB = capsuleCollider.child(s_capsuleColMtxB).asMatrix();
            capsuleCol_pB = capsuleCol_mB.getTranslation(MSpace::kWorld);
            capsuleCol_mA.getScale(capsuleCol_s, MSpace::kWorld);
            capsuleCol_rA = capsuleCollider.child(s_capsuleColRadA).asDouble() * capsuleCol_s[2];
            capsuleCol_rB = capsuleCollider.child(s_capsuleColRadB).asDouble() * capsuleCol_s[2];
                
            const double h = (capsuleCol_pB - capsuleCol_pA).length();
            const MVector ab = (capsuleCol_pB - capsuleCol_pA).normal();

            const double t = ab * (nextPosition - capsuleCol_pA);
            const double ratio = t / h;
            if (ratio <= 0)
            {
                v = nextPosition - capsuleCol_pA;
                r = capsuleCol_rA + initialPose.radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleCol_pA + (v.normal() * r);
                };
            }
            else if (ratio >= 1)
            {
                v = nextPosition - capsuleCol_pB;
                r = capsuleCol_rB + initialPose.radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleCol_pB + (v.normal() * r);
                };
            }
            else
            {
                const MVector q = capsuleCol_pA + (ab * t);
                v = nextPosition - q;
                r = initialPose.radius + (capsuleCol_rA * (1.0 - ratio) + capsuleCol_rB * ratio);
                if (v * v < r * r)
                {
                    nextPosition = q + (v.normal() * r);
                };
            };
        };

        //capsule collision (*Using one center matrix and height)
        for (unsigned int i = 0; i < cciCount; i++) {
            capsuleColiderInputArrayHandle.jumpToArrayElement(i);
            MDataHandle& capsuleColliderInput = capsuleColiderInputArrayHandle.inputValue();
            
            capsuleColMatrix = capsuleColliderInput.child(s_capsuleColliderMatrix).asMatrix();
            
            const MTransformationMatrix capsuleColTransfomMatrix(capsuleColMatrix);
            capsuleColTransfomMatrix.getScale(capsuleColScale, MSpace::kWorld);

            capsuleColHeight = capsuleColliderInput.child(s_capsuleColliderHeight).asDouble();
            
            capsuleColPointA = MVector(MPoint(0, capsuleColHeight * 0.5, 0) * capsuleColMatrix);
            capsuleColPointB = MVector(MPoint(0, -capsuleColHeight * 0.5, 0) * capsuleColMatrix);

            capsuleColRadiusA = capsuleColliderInput.child(s_capsuleColliderRadiusA).asDouble() * capsuleColScale[2];
            capsuleColRadiusB = capsuleColliderInput.child(s_capsuleColliderRadiusB).asDouble() * capsuleColScale[2];

            const double h = (capsuleColPointB - capsuleColPointA).length();
            const MVector ab = (capsuleColPointB - capsuleColPointA).normal();

            const double t = ab * (nextPosition - capsuleColPointA);
            const double ratio = t / h;

            if (ratio <= 0)
            {
                v = nextPosition - capsuleColPointA;
                r = capsuleColRadiusA + initialPose.radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleColPointA + (v.normal() * r);
                };
            }
            else if (ratio >= 1)
            {
                v = nextPosition - capsuleColPointB;
                r = capsuleColRadiusB + initialPose.radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleColPointB + (v.normal() * r);
                };
            }
            else
            {
                const MVector q = capsuleColPointA + (ab * t);
                v = nextPosition - q;
                r = initialPose.radius + (capsuleColRadiusA * (1.0 - ratio) + capsuleColRadiusB * ratio);
                if (v * v < r * r)
                {
                    nextPosition = q + (v.normal() * r);
                };
            };
        };

        //infinite plane collision
        for (unsigned int i = 0; i < pcCount; i++) {
            iPlaneColArrayHandle.jumpToArrayElement(i);
            MDataHandle& iPlaneCollider = iPlaneColArrayHandle.inputValue();
            iPlaneCol_m = iPlaneCollider.child(s_iPlaneColMtx).asMatrix();
            iPlaneCol_p = iPlaneCol_m.getTranslation(MSpace::kWorld);
            iPlaneCol_n = MVector::yAxis.transformAsNormal(iPlaneCol_m.asMatrix()); // Specified by y-axis
                
            const double distancePointPlane = iPlaneCol_n * (nextPosition - iPlaneCol_p);
            if (distancePointPlane - initialPose.radius < 0)
            {
                nextPosition = nextPosition - (iPlaneCol_n * (distancePointPlane - initialPose.radius));
            };
        };

        //mesh collision (experimental)
        for (unsigned int i = 0; i < mcCount; i++) {
            meshColArrayHandle.jumpToArrayElement(i);
            MDataHandle& meshCollider = meshColArrayHandle.inputValue();
            meshCol = meshCollider.asMesh();

            // Get the closest point and closest normal on a mesh
            MPoint closestPoint;
            MVector closestNormal;
            getClosestPoint(meshCol, MPoint(nextPosition), closestPoint, closestNormal);
            closestNormal = closestNormal.normal();

            // Get the vector from the closest point to the current point
            const MVector contactVec = nextPosition - MVector(closestPoint);

            // Get the distance to the surface
            const double distanceContactPoint = closestNormal * contactVec;
            
            // If it is penetrated in the surface, it will be pushed out
            if (distanceContactPoint - initialPose.radius < 0.0) {
                // If it is completely penetrated, it uses the closest normal to push out, if it is only in contact, it uses the contactVec to push out.
                MVector pushOutPos = MVector(closestPoint) + ((distanceContactPoint < 0) ? closestNormal : contactVec.normal()) * initialPose.radius;
                
                // If the position after push out is within meshColCutoff from the current position, it will be pushed out.
                if ((nextPosition - pushOutPos).length() < meshColCutoff) {
                    nextPosition = pushOutPos;
                }
            }
        };
            
        //ground collision
        if (enableGroundCol)
        {
            nextPosition[1] = nextPosition[1] < (groundHeight + initialPose.radius) ? (groundHeight + initialPose.radius) : nextPosition[1]; // Y-Up only
        };

        // angle limit
        if (enableAngleLimit)
        {
            angleLimit(
                initialPose.boneWorldTranslate, 
                initialPose.endWorldTranslate, 
                nextPosition, 
                mathUtils::degToRad(angleLimitDegrees / 2.0)
            );
        };
    }

    // angle limit
    if (iter == 0 && enableAngleLimit)
    {
        angleLimit(
            initialPose.boneWorldTranslate,
            initialPose.endWorldTranslate,
            nextPosition,
            mathUtils::degToRad(angleLimitDegrees / 2.0)
        );
    }

    // distance constraint
    distanceConstraint(initialPose.boneWorldTranslate, nextPosition, initialPose.distance);
    
    // update velocity, position, offset matrix
    m_velocity = (nextPosition - m_position) / dt;
    m_position = nextPosition;
    m_prevOffsetMatrix = initialPose.offsetMatrix;

    // output local euler rotation
    const MVector initVec = (initialPose.endWorldTranslate - initialPose.boneWorldTranslate) * initialPose.boneInitialParentInverseMatrix;
    const MVector targetVec = (nextPosition - initialPose.boneWorldTranslate) * initialPose.boneInitialParentInverseMatrix;
    const MQuaternion quat(initVec, targetVec);
    const MTransformationMatrix offseted_rotation = quat.asMatrix() * initialPose.roMatrix;
    const MQuaternion offseted_quat = offseted_rotation.rotation();
    const MEulerRotation rot = offseted_quat.asEulerRotation();
    
    outputRotateHandle.set3Double(rot.x, rot.y, rot.z);
    outputRotateHandle.setClean();

    return MS::kSuccess;
}
