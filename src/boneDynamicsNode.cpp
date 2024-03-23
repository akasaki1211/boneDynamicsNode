#include "boneDynamicsNode.h"

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

MTypeId boneDynamicsNode::s_id(0x7b001);

MObject boneDynamicsNode::s_enable;

MObject boneDynamicsNode::s_time;
MObject boneDynamicsNode::s_resetTime;
MObject boneDynamicsNode::s_fps;

MObject boneDynamicsNode::s_offsetMatrix;

MObject boneDynamicsNode::s_boneTranslate;
MObject boneDynamicsNode::s_boneJointOrient;
MObject boneDynamicsNode::s_boneParentMatrix;
MObject boneDynamicsNode::s_boneParentInverseMatrix;
MObject boneDynamicsNode::s_endTranslate;

MObject boneDynamicsNode::s_rotationOffset;

MObject boneDynamicsNode::s_boneScale;
MObject boneDynamicsNode::s_boneInverseScale;
MObject boneDynamicsNode::s_endScale;

MObject boneDynamicsNode::s_radius;

MObject boneDynamicsNode::s_damping;
MObject boneDynamicsNode::s_elasticity;
MObject boneDynamicsNode::s_stiffness;
MObject boneDynamicsNode::s_mass;
MObject boneDynamicsNode::s_gravityMultiply;
MObject boneDynamicsNode::s_gravity;

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

MObject boneDynamicsNode::s_outputRotate;

boneDynamicsNode::boneDynamicsNode()
{
    m_init = true;
}

boneDynamicsNode::~boneDynamicsNode()
{
}

void* boneDynamicsNode::creator()
{
    return new boneDynamicsNode;
}

boneDynamicsNode::SchedulingType boneDynamicsNode::schedulingType() const
{
    return kParallel;
}

MStatus boneDynamicsNode::initialize()
{
    MFnNumericAttribute nAttr;
    MFnCompoundAttribute cmpAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;
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

    s_radius = nAttr.create("radius", "r", MFnNumericData::kDouble, 0.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

    // dynamics attributes
    s_damping = nAttr.create("damping", "damp", MFnNumericData::kDouble, 0.1);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(1);

    s_elasticity = nAttr.create("elasticity", "elas", MFnNumericData::kDouble, 30.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);

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

    addAttribute(s_boneTranslate);
    addAttribute(s_boneJointOrient);
    addAttribute(s_boneParentMatrix);
    addAttribute(s_boneParentInverseMatrix);
    addAttribute(s_boneScale);
    addAttribute(s_boneInverseScale);

    addAttribute(s_endTranslate);
    addAttribute(s_endScale);

    addAttribute(s_rotationOffset);

    addAttribute(s_radius);

    addAttribute(s_damping);
    addAttribute(s_elasticity);
    addAttribute(s_stiffness);
    addAttribute(s_mass);
    addAttribute(s_gravity);
    addAttribute(s_gravityMultiply);

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
    
    addAttribute(s_outputRotate);
    

    // attributeAffects
    attributeAffects(s_enable, s_outputRotate);

    attributeAffects(s_time, s_outputRotate);
    attributeAffects(s_resetTime, s_outputRotate);
    attributeAffects(s_fps, s_outputRotate);

    attributeAffects(s_offsetMatrix, s_outputRotate);

    attributeAffects(s_boneTranslate, s_outputRotate);
    attributeAffects(s_boneJointOrient, s_outputRotate);
    attributeAffects(s_boneParentMatrix, s_outputRotate);
    attributeAffects(s_boneParentInverseMatrix, s_outputRotate);
    attributeAffects(s_endTranslate, s_outputRotate);

    attributeAffects(s_rotationOffset, s_outputRotate);

    attributeAffects(s_boneScale, s_outputRotate);
    attributeAffects(s_boneInverseScale, s_outputRotate);
    attributeAffects(s_endScale, s_outputRotate);
    
    attributeAffects(s_radius, s_outputRotate);

    attributeAffects(s_damping, s_outputRotate);
    attributeAffects(s_elasticity, s_outputRotate);
    attributeAffects(s_stiffness, s_outputRotate);
    attributeAffects(s_mass, s_outputRotate);
    attributeAffects(s_gravity, s_outputRotate);
    attributeAffects(s_gravityMultiply, s_outputRotate);

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

MStatus boneDynamicsNode::compute(const MPlug& plug, MDataBlock& data)
{
    if (plug != s_outputRotate)
    {
        return MS::kUnknownParameter;
    }

    // output data handles
    MDataHandle outputRotateHandle = data.outputValue(s_outputRotate);
    
    bool enable = data.inputValue(s_enable).asBool();
    if (!enable)
    {
        //return MS::kUnknownParameter;
        outputRotateHandle.set3Double(0.0, 0.0, 0.0);

        data.setClean(plug);

        return MS::kSuccess;
    }

    // input values
    MMatrix offsetMatrix = data.inputValue(s_offsetMatrix).asMatrix();

    MVector boneTranslate = data.inputValue(s_boneTranslate).asVector();
    MVector boneJointOrient = data.inputValue(s_boneJointOrient).asVector();
    MMatrix boneParentMatrix = data.inputValue(s_boneParentMatrix).asMatrix();
    MMatrix boneParentInverseMatrix = data.inputValue(s_boneParentInverseMatrix).asMatrix();
    MVector boneScale = data.inputValue(s_boneScale).asVector();
    MVector boneInverseScaleInput = data.inputValue(s_boneInverseScale).asVector();
    MVector boneInverseScale(1.0 / boneInverseScaleInput.x, 1.0 / boneInverseScaleInput.y, 1.0 / boneInverseScaleInput.z);
    
    MVector endTranslate = data.inputValue(s_endTranslate).asVector();
    MVector endScale = data.inputValue(s_endScale).asVector();

    MVector rotationOffset = data.inputValue(s_rotationOffset).asVector();

    // get rotation offset matrix
    MEulerRotation rotationOffsetEuler(rotationOffset, MEulerRotation::RotationOrder::kXYZ); // Specified xyz
    MMatrix roMatrix = rotationOffsetEuler.asMatrix();
    MMatrix roInverseMatrix = roMatrix.inverse();

    // get joint orient matrix
    MEulerRotation boneJointOrientEuler(boneJointOrient, MEulerRotation::RotationOrder::kXYZ); // Specified xyz
    MMatrix joMatrix = boneJointOrientEuler.asMatrix();
    MMatrix joInverseMatrix = joMatrix.inverse();

    // get bone matrix
    MTransformationMatrix boneTransformationMatrix;
    boneTransformationMatrix.setTranslation(boneTranslate, MSpace::kWorld);
    MMatrix boneMatrix = boneTransformationMatrix.asMatrix();
    
    // calc bone world position
    MPoint boneWorldTranslatePoint = MPoint(boneTranslate) * boneParentMatrix;
    MVector boneWorldTranslate(boneWorldTranslatePoint);

    // calc end world position
    MPoint scaledEndTranslate(
        endTranslate.x * boneScale.x * boneInverseScale.x, 
        endTranslate.y * boneScale.y * boneInverseScale.y, 
        endTranslate.z * boneScale.z * boneInverseScale.z
    );
    MPoint endWorldTranslatePoint = scaledEndTranslate * roMatrix * joMatrix * boneMatrix * boneParentMatrix;
    MVector endWorldTranslate(endWorldTranslatePoint);

    // radius
    double radius = data.inputValue(s_radius).asDouble();
    MTransformationMatrix boneParentTransformationMatrix(boneParentMatrix);
    double boneParentScale[3];
    boneParentTransformationMatrix.getScale(boneParentScale, MSpace::kWorld);
    radius *= endScale.z * boneInverseScale.z * boneParentScale[2]; // Specified by z-axis

    // bone length
    double distance = (endWorldTranslate - boneWorldTranslate).length();

    // dynamics
    double damping = data.inputValue(s_damping).asDouble();
    double elasticity = data.inputValue(s_elasticity).asDouble();
    double stiffness = data.inputValue(s_stiffness).asDouble();
    double mass = data.inputValue(s_mass).asDouble();
    double gravityMultiply = data.inputValue(s_gravityMultiply).asDouble();
    MVector gravity = data.inputValue(s_gravity).asVector();
    
    MTime time = data.inputValue(s_time).asTime();
    MTime resetTime = data.inputValue(s_resetTime).asTime();
    
    // reset(init)
    if (time <= resetTime || m_init) {
        m_prevOffsetMatrix = MMatrix();
        m_position = endWorldTranslate;
        m_velocity = MVector(0.0, 0.0, 0.0);
        m_init = false;
        
        outputRotateHandle.set3Double(0.0, 0.0, 0.0);

        data.setClean(plug);
        
        return MS::kSuccess;
    }

    // delta time
    //double dt = 1.0 / getFPS();
    double fps = data.inputValue(s_fps).asDouble();
    double dt = 1.0 / fps;

    // position offset
    MPoint offsetedPosition = MPoint(m_position) * m_prevOffsetMatrix.inverse() * offsetMatrix;
    m_position = MVector(offsetedPosition);

    // velocity damping
    m_velocity = m_velocity * (1.0 - damping);

    // add velocity
    m_nextPosition = m_position + (m_velocity * dt);

    // external force
    MVector external_force;
    // spring
    external_force += ((endWorldTranslate - m_nextPosition) * elasticity) / mass;
    // gravity
    external_force += gravity * gravityMultiply;
    // add external force
    m_nextPosition += external_force * dt * dt;
    
    // keep position
    m_nextPosition += (m_position - m_nextPosition) * stiffness;

    // collision
    long iter = data.inputValue(s_iterations).asLong();

    bool enableGroundCol = data.inputValue(s_enableGroundCol).asBool();
    double groundHeight = data.inputValue(s_groundHeight).asDouble();

    MArrayDataHandle sphereColArrayHandle = data.inputArrayValue(s_sphereCollider);
    unsigned int scCount = sphereColArrayHandle.elementCount();
        
    MArrayDataHandle capsuleColArrayHandle = data.inputArrayValue(s_capsuleCollider);
    unsigned int ccCount = capsuleColArrayHandle.elementCount();
        
    MArrayDataHandle iPlaneColArrayHandle = data.inputArrayValue(s_iPlaneCollider);
    unsigned int pcCount = iPlaneColArrayHandle.elementCount();

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
    MVector iPlaneCol_n = MVector::yAxis;

    MVector v;
    double r;

    for (int i = 0; i < iter; i++)
    {
        //sphere collision
        for (unsigned int i = 0; i < scCount; i++) {
            sphereColArrayHandle.jumpToElement(i);
            MDataHandle sphereCollider = sphereColArrayHandle.inputValue();
            sphereCol_m = sphereCollider.child(s_sphereColMtx).asMatrix();
            sphereCol_p = sphereCol_m.getTranslation(MSpace::kWorld);
            sphereCol_m.getScale(sphereCol_s, MSpace::kWorld);
            sphereCol_r = sphereCollider.child(s_sphereColRad).asDouble() * sphereCol_s[2];
                
            v = m_nextPosition - sphereCol_p;
            r = sphereCol_r + radius;
            if (v * v < r * r)
            {
                m_nextPosition = sphereCol_p + (v.normal() * r);
            };
        };

        //capsule collision
        for (unsigned int i = 0; i < ccCount; i++) {
            capsuleColArrayHandle.jumpToElement(i);
            MDataHandle capsuleCollider = capsuleColArrayHandle.inputValue();
            capsuleCol_mA = capsuleCollider.child(s_capsuleColMtxA).asMatrix();
            capsuleCol_pA = capsuleCol_mA.getTranslation(MSpace::kWorld);
            capsuleCol_mB = capsuleCollider.child(s_capsuleColMtxB).asMatrix();
            capsuleCol_pB = capsuleCol_mB.getTranslation(MSpace::kWorld);
            capsuleCol_mA.getScale(capsuleCol_s, MSpace::kWorld);
            capsuleCol_rA = capsuleCollider.child(s_capsuleColRadA).asDouble() * capsuleCol_s[2];
            capsuleCol_rB = capsuleCollider.child(s_capsuleColRadB).asDouble() * capsuleCol_s[2];
                
            double h = (capsuleCol_pB - capsuleCol_pA).length();
            MVector ab = (capsuleCol_pB - capsuleCol_pA).normal();

            double t = ab * (m_nextPosition - capsuleCol_pA);
            double ratio = t / h;
            if (ratio <= 0)
            {
                v = m_nextPosition - capsuleCol_pA;
                r = capsuleCol_rA + radius;
                if (v * v < r * r)
                {
                    m_nextPosition = capsuleCol_pA + (v.normal() * r);
                };
            }
            else if (ratio >= 1)
            {
                v = m_nextPosition - capsuleCol_pB;
                r = capsuleCol_rB + radius;
                if (v * v < r * r)
                {
                    m_nextPosition = capsuleCol_pB + (v.normal() * r);
                };
            }
            else
            {
                MVector q = capsuleCol_pA + (ab * t);
                v = m_nextPosition - q;
                r = radius + (capsuleCol_rA * (1.0 - ratio) + capsuleCol_rB * ratio);
                if (v * v < r * r)
                {
                    m_nextPosition = q + (v.normal() * r);
                };
            };
        };

        //infinite plane collision
        for (unsigned int i = 0; i < pcCount; i++) {
            iPlaneColArrayHandle.jumpToElement(i);
            MDataHandle iPlaneCollider = iPlaneColArrayHandle.inputValue();
            iPlaneCol_m = iPlaneCollider.child(s_iPlaneColMtx).asMatrix();
            iPlaneCol_p = iPlaneCol_m.getTranslation(MSpace::kWorld);
            iPlaneCol_n = MVector::yAxis.transformAsNormal(iPlaneCol_m.asMatrix());
                
            double distancePointPlane = iPlaneCol_n * (m_nextPosition - iPlaneCol_p);
            if (distancePointPlane - radius < 0)
            {
                m_nextPosition = m_nextPosition - (iPlaneCol_n * (distancePointPlane - radius));
            };
        };
            
        //ground collision
        if (enableGroundCol)
        {
            m_nextPosition[1] = m_nextPosition[1] < (groundHeight + radius) ? (groundHeight + radius) : m_nextPosition[1];
        };

        // distance constraint
        m_nextPosition = boneWorldTranslate + ((m_nextPosition - boneWorldTranslate).normal() * distance);
    }

    // distance constraint
    m_nextPosition = boneWorldTranslate + ((m_nextPosition - boneWorldTranslate).normal() * distance);
    
    // update velocity
    m_velocity = (m_nextPosition - m_position) / dt;

    // update position
    m_position = m_nextPosition;
    
    // update offset matrix
    m_prevOffsetMatrix = offsetMatrix;

    // set output local euler rotation
    MVector initVec = (endWorldTranslate - boneWorldTranslate) * boneParentInverseMatrix * joInverseMatrix * roInverseMatrix;
    MVector targetVec = (m_nextPosition - boneWorldTranslate) * boneParentInverseMatrix * joInverseMatrix * roInverseMatrix;
    MQuaternion quat(initVec, targetVec);
    MTransformationMatrix offseted_rotation = quat.asMatrix() * roMatrix;
    MQuaternion offseted_quat = offseted_rotation.rotation();
    MEulerRotation rot = offseted_quat.asEulerRotation();
    
    outputRotateHandle.set3Double(rot.x, rot.y, rot.z);
        
    data.setClean(plug);

    return MS::kSuccess;
}