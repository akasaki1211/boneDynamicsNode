#include "boneDynamicsNode.h"

#include <algorithm>
#include <cassert>

MTypeId boneDynamicsNode::s_id(0x7b001);

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
MObject boneDynamicsNode::m_turbulenceVectorChangeScale;
MObject boneDynamicsNode::m_turbulenceVectorChangeMax;

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

/* -------------------------------
Simulation Support
    references: 
    https://damassets.autodesk.net/content/dam/autodesk/www/html/maya-cached-playback/2026/MayaCachedPlaybackWhitePaper.html#dynamics-and-layered-caching
    https://damassets.autodesk.net/content/dam/autodesk/www/html/maya-cached-playback/2026/MayaCachedPlaybackWhitePaper.html#plug-in-authoring
------------------------------- */
void boneDynamicsNode::getCacheSetup(const MEvaluationNode& evalNode, MNodeCacheDisablingInfo& disablingInfo, MNodeCacheSetupInfo& cacheSetupInfo, MObjectArray& monitoredAttributes) const
{
    MPxNode::getCacheSetup(evalNode, disablingInfo, cacheSetupInfo, monitoredAttributes);
    assert(!disablingInfo.getCacheDisabled());
    cacheSetupInfo.setRequirement(MNodeCacheSetupInfo::kSimulationSupport, true);
}

MTimeRange boneDynamicsNode::transformInvalidationRange(
    const MPlug& source,
    const MTimeRange& input
) const
{
    static constexpr MTime::MTick kMaximumTimeTick = std::numeric_limits<MTime::MTick>::max() / 2;
    static constexpr MTime::MTick kMinimumTimeTick = std::numeric_limits<MTime::MTick>::min() / 2 + 1;
    static const MTime kMaximumTime{ kMaximumTimeTick / static_cast<double>(MTime::ticksPerSecond()), MTime::kSeconds };
    static const MTime kMinimumTime{ kMinimumTimeTick / static_cast<double>(MTime::ticksPerSecond()), MTime::kSeconds };

    MDataBlock data = const_cast<boneDynamicsNode*>(this)->forceCache();
    if (!data.isClean(s_resetTime) || !data.isClean(s_enable))
    {
        return MTimeRange{ kMinimumTime, kMaximumTime };
    }

    const bool enable = data.inputValue(s_enable).asBool();
    if (!enable)
    {
        return input;
    }
    
    const MTime& resetTime = data.inputValue(s_resetTime).asTime();
    if (input.intersects(resetTime, kMaximumTime))
    {
        return input | MTimeRange{ resetTime, kMaximumTime };
    }
    else
    {
        return MTimeRange{};
    }
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

    //s_fps = nAttr.create("fps", "fps", MFnNumericData::kDouble, getFPS());
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

    m_turbulenceVectorChangeScale = nAttr.create("turbulenceVectorChangeScale", "wvcs", MFnNumericData::kDouble, 0.05);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    m_turbulenceVectorChangeMax = nAttr.create("turbulenceVectorChangeMax", "wvcm", MFnNumericData::kDouble, 0.1);
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
    s_radius = nAttr.create("radius", "r", MFnNumericData::kDouble, 0.0);
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
    addAttribute(s_enable);

    addAttribute(s_time);
    addAttribute(s_resetTime);
    addAttribute(s_fps);

    addAttribute(s_offsetMatrix);
    addAttribute(s_offsetMatrixWeight);

    addAttribute(s_boneTranslate);
    addAttribute(s_boneJointOrient);
    addAttribute(s_boneParentMatrix);
    addAttribute(s_boneParentInverseMatrix);
    addAttribute(s_boneScale);
    addAttribute(s_boneInverseScale);

    addAttribute(s_endTranslate);
    addAttribute(s_endScale);

    addAttribute(s_rotationOffset);

    addAttribute(s_damping);
    addAttribute(s_elasticity);
    addAttribute(s_elasticForceFunction);
    addAttribute(s_stiffness);
    addAttribute(s_mass);
    addAttribute(s_gravity);
    addAttribute(s_gravityMultiply);
    addAttribute(s_additionalForce);
    addAttribute(s_additionalForceScale);
    
    addAttribute(s_enableTurbulence);
    addAttribute(s_turbulenceSeed);
    addAttribute(s_turbulenceStrength);
    addAttribute(m_turbulenceVectorChangeScale);
    addAttribute(m_turbulenceVectorChangeMax);

    addAttribute(s_enableAngleLimit);
    addAttribute(s_angleLimit);

    addAttribute(s_radius);

    addAttribute(s_iterations);

    addAttribute(s_enableGroundCol);
    addAttribute(s_groundHeight);
    addAttribute(s_sphereColMtx);
    addAttribute(s_sphereColRad);
    addAttribute(s_sphereCollider);
    addAttribute(s_capsuleColMtxA);
    addAttribute(s_capsuleColMtxB);
    addAttribute(s_capsuleColRadA);
    addAttribute(s_capsuleColRadB);
    addAttribute(s_capsuleCollider);
    addAttribute(s_iPlaneColMtx);
    addAttribute(s_iPlaneCollider);
    addAttribute(s_meshCollider);
    addAttribute(s_meshColCutoff);

    addAttribute(s_outputRotate);
    

    // attributeAffects
    attributeAffects(s_enable, s_outputRotate);

    attributeAffects(s_time, s_outputRotate);
    attributeAffects(s_resetTime, s_outputRotate);
    attributeAffects(s_fps, s_outputRotate);

    attributeAffects(s_offsetMatrix, s_outputRotate);
    attributeAffects(s_offsetMatrixWeight, s_outputRotate);

    attributeAffects(s_boneTranslate, s_outputRotate);
    attributeAffects(s_boneJointOrient, s_outputRotate);
    attributeAffects(s_boneParentMatrix, s_outputRotate);
    attributeAffects(s_boneParentInverseMatrix, s_outputRotate);
    attributeAffects(s_endTranslate, s_outputRotate);

    attributeAffects(s_rotationOffset, s_outputRotate);

    attributeAffects(s_boneScale, s_outputRotate);
    attributeAffects(s_boneInverseScale, s_outputRotate);
    attributeAffects(s_endScale, s_outputRotate);
    
    attributeAffects(s_damping, s_outputRotate);
    attributeAffects(s_elasticity, s_outputRotate);
    attributeAffects(s_elasticForceFunction, s_outputRotate);
    attributeAffects(s_stiffness, s_outputRotate);
    attributeAffects(s_mass, s_outputRotate);
    attributeAffects(s_gravity, s_outputRotate);
    attributeAffects(s_gravityMultiply, s_outputRotate);
    attributeAffects(s_additionalForce, s_outputRotate);
    attributeAffects(s_additionalForceScale, s_outputRotate);

    attributeAffects(s_enableTurbulence, s_outputRotate);
    attributeAffects(s_turbulenceSeed, s_outputRotate);
    attributeAffects(s_turbulenceStrength, s_outputRotate);
    attributeAffects(m_turbulenceVectorChangeScale, s_outputRotate);
    attributeAffects(m_turbulenceVectorChangeMax, s_outputRotate);

    attributeAffects(s_enableAngleLimit, s_outputRotate);
    attributeAffects(s_angleLimit, s_outputRotate);

    attributeAffects(s_radius, s_outputRotate);

    attributeAffects(s_iterations, s_outputRotate);

    attributeAffects(s_enableGroundCol, s_outputRotate);
    attributeAffects(s_groundHeight, s_outputRotate);
    attributeAffects(s_sphereColMtx, s_outputRotate);
    attributeAffects(s_sphereColRad, s_outputRotate);
    attributeAffects(s_sphereCollider, s_outputRotate);
    attributeAffects(s_capsuleColMtxA, s_outputRotate);
    attributeAffects(s_capsuleColMtxB, s_outputRotate);
    attributeAffects(s_capsuleColRadA, s_outputRotate);
    attributeAffects(s_capsuleColRadB, s_outputRotate);
    attributeAffects(s_capsuleCollider, s_outputRotate);
    attributeAffects(s_iPlaneColMtx, s_outputRotate);
    attributeAffects(s_iPlaneCollider, s_outputRotate);
    attributeAffects(s_meshCollider, s_outputRotate);
    attributeAffects(s_meshColCutoff, s_outputRotate);

    return MS::kSuccess;
}

/*double boneDynamicsNode::getFPS()
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

double boneDynamicsNode::degToRad(double deg)
{
    return deg * (M_PI / 180);
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

uint32_t boneDynamicsNode::rotl(const uint32_t x, int k) {
    return (x << k) | (x >> (32 - k));
}

double boneDynamicsNode::rand_double(const double scale) {

    // xoshiro128++

    uint32_t* s = m_rngState;
    uint32_t result = rotl(s[0] + s[3], 7) + s[0];
    uint32_t t = s[1] << 9;

    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];

    s[2] ^= t;

    s[3] = rotl(s[3], 11);

    // normalize
    double normalized = result * 2.3283064365386963e-10;
    
    // scale
    return normalized * (scale * 2) - scale;
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
    
    const bool enable = data.inputValue(s_enable).asBool();
    if (!enable)
    {
        //return MS::kUnknownParameter;
        outputRotateHandle.set3Double(0.0, 0.0, 0.0);

        data.setClean(plug);

        return MS::kSuccess;
    }

    // offset matrix
    const MMatrix& offsetMatrix = data.inputValue(s_offsetMatrix).asMatrix();
    const double offsetMatrixWeight = data.inputValue(s_offsetMatrixWeight).asDouble();

    // bone
    const MVector& boneTranslate = data.inputValue(s_boneTranslate).asVector();
    const MVector& boneJointOrient = data.inputValue(s_boneJointOrient).asVector();
    const MMatrix& boneParentMatrix = data.inputValue(s_boneParentMatrix).asMatrix();
    const MMatrix& boneParentInverseMatrix = data.inputValue(s_boneParentInverseMatrix).asMatrix();
    const MVector& boneScale = data.inputValue(s_boneScale).asVector();
    const MVector& boneInverseScaleInput = data.inputValue(s_boneInverseScale).asVector();
    const MVector boneInverseScale(1.0 / boneInverseScaleInput.x, 1.0 / boneInverseScaleInput.y, 1.0 / boneInverseScaleInput.z);
    
    // end
    const MVector& endTranslate = data.inputValue(s_endTranslate).asVector();
    const MVector& endScale = data.inputValue(s_endScale).asVector();

    // rotation offset
    const MVector& rotationOffset = data.inputValue(s_rotationOffset).asVector();

    // rotation offset matrix
    const MEulerRotation rotationOffsetEuler(rotationOffset, ROTATION_ORDER);
    const MMatrix roMatrix = rotationOffsetEuler.asMatrix();
    const MMatrix roInverseMatrix = !roMatrix.isEquivalent(MMatrix::identity) ? roMatrix.inverse() : MMatrix::identity;

    // joint orient matrix
    const MMatrix joMatrix = MEulerRotation(boneJointOrient, ROTATION_ORDER).asMatrix();
    const MMatrix joInverseMatrix = !joMatrix.isEquivalent(MMatrix::identity) ? joMatrix.inverse() : MMatrix::identity;

    // bone matrix
    MTransformationMatrix boneTransformationMatrix;
    boneTransformationMatrix.setTranslation(boneTranslate, MSpace::kWorld);
    const MMatrix boneMatrix = boneTransformationMatrix.asMatrix();
    
    // bone world position
    const MPoint boneWorldTranslatePoint = MPoint(boneTranslate) * boneParentMatrix;
    const MVector boneWorldTranslate(boneWorldTranslatePoint);

    // end world position
    const MPoint scaledEndTranslate(
        endTranslate.x * boneScale.x * boneInverseScale.x, 
        endTranslate.y * boneScale.y * boneInverseScale.y, 
        endTranslate.z * boneScale.z * boneInverseScale.z
    );
    const MPoint endWorldTranslatePoint = scaledEndTranslate * roMatrix * joMatrix * boneMatrix * boneParentMatrix;
    const MVector endWorldTranslate(endWorldTranslatePoint);

    // radius
    double radius = data.inputValue(s_radius).asDouble();
    const MTransformationMatrix boneParentTransformationMatrix(boneParentMatrix);
    double boneParentScale[3];
    boneParentTransformationMatrix.getScale(boneParentScale, MSpace::kWorld);
    radius *= endScale.z * boneInverseScale.z * boneParentScale[2]; // Specified by scale Z

    // bone length
    const double distance = (endWorldTranslate - boneWorldTranslate).length();

    // dynamic parameters
    const double damping = data.inputValue(s_damping).asDouble();
    const double elasticity = data.inputValue(s_elasticity).asDouble();
    const double stiffness = data.inputValue(s_stiffness).asDouble();
    const double mass = data.inputValue(s_mass).asDouble();
    const MVector& gravity = data.inputValue(s_gravity).asVector();
    const double gravityMultiply = data.inputValue(s_gravityMultiply).asDouble();
    const MVector& additionalForce = data.inputValue(s_additionalForce).asVector();
    const double additionalForceScale = data.inputValue(s_additionalForceScale).asDouble();
    
    const MTime& time = data.inputValue(s_time).asTime();
    const MTime& resetTime = data.inputValue(s_resetTime).asTime();

    const bool enableTurbulence = data.inputValue(s_enableTurbulence).asBool();
    
    // reset and initialization
    if (time <= resetTime || m_init) {
        m_prevOffsetMatrix = offsetMatrix;
        m_position = endWorldTranslate;
        m_velocity = MVector();

        if (enableTurbulence) {
            m_turbulenceVector = MVector();
            m_turbulenceVectorChange = MVector();
            m_lastSeed = -1;
            m_lastFrame = -1;
            m_rngState[0] = 0;
            m_rngState[1] = 0;
            m_rngState[2] = 0;
            m_rngState[3] = 0;
        }

        m_init = false;
        
        outputRotateHandle.set3Double(rotationOffsetEuler.x, rotationOffsetEuler.y, rotationOffsetEuler.z);

        data.setClean(plug);
        
        return MS::kSuccess;
    }

    // delta time
    //double dt = 1.0 / getFPS();
    const double fps = data.inputValue(s_fps).asDouble();
    const double dt = 1.0 / fps;
    const double dt2 = dt * dt;

    // position offset
    const MPoint offsetedPosition = MPoint(m_position) * m_prevOffsetMatrix.inverse() * offsetMatrix;
    m_position = MVector(offsetedPosition) * offsetMatrixWeight + m_position * (1.0 - offsetMatrixWeight);
    
    // velocity damping
    m_velocity *= (1.0 - damping);
    
    // add velocity
    MVector nextPosition = m_position + (m_velocity * dt);

    // turbulence vector
    if (enableTurbulence) {
        int frame = static_cast<int>(time.as(MTime::uiUnit()));
        int turbulenceSeed = data.inputValue(s_turbulenceSeed).asInt();

        if (turbulenceSeed != m_lastSeed || frame != m_lastFrame) {

            // Init xoshiro128++
            uint32_t s = static_cast<uint32_t>(turbulenceSeed) ^ (static_cast<uint32_t>(frame) * 0x9e3779b9u);
            for (int i = 0; i < 4; ++i) {
                s = s * 1664525u + 1013904223u;
                m_rngState[i] = s;
            }
            m_lastSeed = turbulenceSeed;
            m_lastFrame = frame;
        }
        
        const double turbulenceVectorChangeScale = data.inputValue(m_turbulenceVectorChangeScale).asDouble();
        const double turbulenceVectorChangeMax = data.inputValue(m_turbulenceVectorChangeMax).asDouble();

        m_turbulenceVectorChange += MVector(
            rand_double(turbulenceVectorChangeScale), 
            rand_double(turbulenceVectorChangeScale), 
            rand_double(turbulenceVectorChangeScale)
        );

        m_turbulenceVectorChange.x = std::max(-turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.x, turbulenceVectorChangeMax));
        m_turbulenceVectorChange.y = std::max(-turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.y, turbulenceVectorChangeMax));
        m_turbulenceVectorChange.z = std::max(-turbulenceVectorChangeMax, std::min(m_turbulenceVectorChange.z, turbulenceVectorChangeMax));

        m_turbulenceVector += m_turbulenceVectorChange;
        m_turbulenceVector.normalize();
    }

    // external force
    // gravity
    MVector externalForce = gravity * gravityMultiply;

    // spring force
    const int elasticForceFunctionValue = data.inputValue(s_elasticForceFunction).asShort();
    const MVector springVector = (endWorldTranslate - nextPosition);
    const double springVectorLength = springVector.length();
    MVector springForce;

    switch (elasticForceFunctionValue) // experimental
    {
        case 0: // Linear
            springForce = (springVector * elasticity) / mass;
            break;
        case 1: // Quadratic
            springForce = (springVector * ((springVectorLength + 1.0) * elasticity)) / mass;
            break;
        case 2: // Cubic
            springForce = (springVector * ((springVectorLength + 1.0) * (springVectorLength + 1.0) * elasticity)) / mass;
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
    externalForce += (additionalForce * additionalForceScale) / mass;
    
    // turbulence force
    if (enableTurbulence) {
        const double turbulenceStrength = data.inputValue(s_turbulenceStrength).asDouble();
        externalForce += (m_turbulenceVector * turbulenceStrength) / mass;
    }

    // apply external force
    nextPosition += externalForce * dt2;
    
    // apply stiffness
    nextPosition += (m_position - nextPosition) * stiffness;

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
        distanceConstraint(boneWorldTranslate, nextPosition, distance);
        
        //sphere collision
        for (unsigned int i = 0; i < scCount; i++) {
            sphereColArrayHandle.jumpToArrayElement(i);
            MDataHandle& sphereCollider = sphereColArrayHandle.inputValue();
            sphereCol_m = sphereCollider.child(s_sphereColMtx).asMatrix();
            sphereCol_p = sphereCol_m.getTranslation(MSpace::kWorld);
            sphereCol_m.getScale(sphereCol_s, MSpace::kWorld);
            sphereCol_r = sphereCollider.child(s_sphereColRad).asDouble() * sphereCol_s[2];
                
            v = nextPosition - sphereCol_p;
            r = sphereCol_r + radius;
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
                r = capsuleCol_rA + radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleCol_pA + (v.normal() * r);
                };
            }
            else if (ratio >= 1)
            {
                v = nextPosition - capsuleCol_pB;
                r = capsuleCol_rB + radius;
                if (v * v < r * r)
                {
                    nextPosition = capsuleCol_pB + (v.normal() * r);
                };
            }
            else
            {
                const MVector q = capsuleCol_pA + (ab * t);
                v = nextPosition - q;
                r = radius + (capsuleCol_rA * (1.0 - ratio) + capsuleCol_rB * ratio);
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
            if (distancePointPlane - radius < 0)
            {
                nextPosition = nextPosition - (iPlaneCol_n * (distancePointPlane - radius));
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
            if (distanceContactPoint - radius < 0.0) {
                // If it is completely penetrated, it uses the closest normal to push out, if it is only in contact, it uses the contactVec to push out.
                MVector pushOutPos = MVector(closestPoint) + ((distanceContactPoint < 0) ? closestNormal : contactVec.normal()) * radius;
                
                // If the position after push out is within meshColCutoff from the current position, it will be pushed out.
                if ((nextPosition - pushOutPos).length() < meshColCutoff) {
                    nextPosition = pushOutPos;
                }
            }
        };
            
        //ground collision
        if (enableGroundCol)
        {
            nextPosition[1] = nextPosition[1] < (groundHeight + radius) ? (groundHeight + radius) : nextPosition[1]; // Y-Up only
        };

        // angle limit
        if (enableAngleLimit)
        {
            angleLimit(
                boneWorldTranslate, 
                endWorldTranslate, 
                nextPosition, 
                degToRad(angleLimitDegrees / 2.0)
            );
        };
    }

    // angle limit
    if (iter == 0 && enableAngleLimit)
    {
        angleLimit(
            boneWorldTranslate,
            endWorldTranslate,
            nextPosition,
            degToRad(angleLimitDegrees / 2.0)
        );
    }

    // distance constraint
    distanceConstraint(boneWorldTranslate, nextPosition, distance);
    
    // update velocity, position, offset matrix
    m_velocity = (nextPosition - m_position) / dt;
    m_position = nextPosition;
    m_prevOffsetMatrix = offsetMatrix;

    // output local euler rotation
    const MVector initVec = (endWorldTranslate - boneWorldTranslate) * boneParentInverseMatrix * joInverseMatrix * roInverseMatrix;
    const MVector targetVec = (nextPosition - boneWorldTranslate) * boneParentInverseMatrix * joInverseMatrix * roInverseMatrix;
    const MQuaternion quat(initVec, targetVec);
    const MTransformationMatrix offseted_rotation = quat.asMatrix() * roMatrix;
    const MQuaternion offseted_quat = offseted_rotation.rotation();
    const MEulerRotation rot = offseted_quat.asEulerRotation();
    
    outputRotateHandle.set3Double(rot.x, rot.y, rot.z);
        
    data.setClean(plug);

    return MS::kSuccess;
}