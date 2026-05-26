#include "boneDynamicsVisualizer.h"
#include "visualizerGeometry.h"
#include "../boneDynamicsUtils.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>


MTypeId boneDynamicsVisualizer::s_id(nodeIds::boneDynamicsVisualizer);

MString boneDynamicsVisualizer::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/visualizer");

MString boneDynamicsVisualizer::s_drawRegistrantId("boneDynamicsNodePlugin");

MObject boneDynamicsVisualizer::s_drawAngleLimit;
MObject boneDynamicsVisualizer::s_drawCollisionRadius;

MObject boneDynamicsVisualizer::s_enable;

MObject boneDynamicsVisualizer::s_outputRotate;

MObject boneDynamicsVisualizer::s_boneTranslate;
MObject boneDynamicsVisualizer::s_boneJointOrient;
MObject boneDynamicsVisualizer::s_boneParentMatrix;
MObject boneDynamicsVisualizer::s_boneParentInverseMatrix;
MObject boneDynamicsVisualizer::s_boneScale;
MObject boneDynamicsVisualizer::s_boneInverseScale;

MObject boneDynamicsVisualizer::s_endTranslate;
MObject boneDynamicsVisualizer::s_endScale;

MObject boneDynamicsVisualizer::s_rotationOffset;

MObject boneDynamicsVisualizer::s_radius;

MObject boneDynamicsVisualizer::s_angleLimit;
MObject boneDynamicsVisualizer::s_angleConeSize;

boneDynamicsVisualizer::boneDynamicsVisualizer() {}

boneDynamicsVisualizer::~boneDynamicsVisualizer() {}

bool boneDynamicsVisualizer::isBounded() const
{
    return false; // Return False
}

MBoundingBox boneDynamicsVisualizer::boundingBox() const
{
    return MBoundingBox();
}

void* boneDynamicsVisualizer::creator()
{
    return new boneDynamicsVisualizer();
}

MStatus boneDynamicsVisualizer::initialize()
{
    MFnNumericAttribute nAttr;
    MFnUnitAttribute uAttr;
    MFnMatrixAttribute mAttr;
    MObject x, y, z;

    s_drawAngleLimit = nAttr.create("drawAngleLimit", "dal", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    s_drawCollisionRadius = nAttr.create("drawCollisionRadius", "dcr", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    s_enable = nAttr.create("enable", "en", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = uAttr.create("outputRotateX", "outrx", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("outputRotateY", "outry", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("outputRotateZ", "outrz", MFnUnitAttribute::kAngle, 0.0);
    s_outputRotate = nAttr.create("outputRotate", "outr", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = nAttr.create("boneTranslateX", "btx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("boneTranslateY", "bty", MFnNumericData::kDouble, 0.0);
    z = nAttr.create("boneTranslateZ", "btz", MFnNumericData::kDouble, 0.0);
    s_boneTranslate = nAttr.create("boneTranslate", "bt", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = uAttr.create("boneJointOrientX", "bjox", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("boneJointOrientY", "bjoy", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("boneJointOrientZ", "bjoz", MFnUnitAttribute::kAngle, 0.0);
    s_boneJointOrient = nAttr.create("boneJointOrient", "bjo", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    s_boneParentMatrix = mAttr.create("boneParentMatrix", "bpmtx");
    mAttr.setKeyable(true);
    mAttr.setAffectsAppearance(true);

    s_boneParentInverseMatrix = mAttr.create("boneParentInverseMatrix", "bpimtx");
    mAttr.setKeyable(true);
    mAttr.setAffectsAppearance(true);

    x = nAttr.create("boneScaleX", "bsx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("boneScaleY", "bsy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("boneScaleZ", "bsz", MFnNumericData::kDouble, 1.0);
    s_boneScale = nAttr.create("boneScale", "bs", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = nAttr.create("boneInverseScaleX", "bisx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("boneInverseScaleY", "bisy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("boneInverseScaleZ", "bisz", MFnNumericData::kDouble, 1.0);
    s_boneInverseScale = nAttr.create("boneInverseScale", "bis", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = nAttr.create("endTranslateX", "etx", MFnNumericData::kDouble, 0.0);
    y = nAttr.create("endTranslateY", "ety", MFnNumericData::kDouble, 0.0);
    z = nAttr.create("endTranslateZ", "etz", MFnNumericData::kDouble, 0.0);
    s_endTranslate = nAttr.create("endTranslate", "et", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = nAttr.create("endScaleX", "esx", MFnNumericData::kDouble, 1.0);
    y = nAttr.create("endScaleY", "esy", MFnNumericData::kDouble, 1.0);
    z = nAttr.create("endScaleZ", "esz", MFnNumericData::kDouble, 1.0);
    s_endScale = nAttr.create("endScale", "es", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    x = uAttr.create("rotationOffsetX", "rox", MFnUnitAttribute::kAngle, 0.0);
    y = uAttr.create("rotationOffsetY", "roy", MFnUnitAttribute::kAngle, 0.0);
    z = uAttr.create("rotationOffsetZ", "roz", MFnUnitAttribute::kAngle, 0.0);
    s_rotationOffset = nAttr.create("rotationOffset", "ro", x, y, z);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    s_radius = nAttr.create("radius", "r", MFnNumericData::kDouble, 1.0); // TODO: Change to MFnUnitAttribute::kDistance
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setAffectsAppearance(true);

    s_angleLimit = nAttr.create("angleLimit", "al", MFnNumericData::kDouble, 60.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(360);
    nAttr.setAffectsAppearance(true);

    s_angleConeSize = nAttr.create("angleConeSize", "acs", MFnNumericData::kDouble, 1.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setAffectsAppearance(true);

    addAttribute(s_drawAngleLimit);
    addAttribute(s_drawCollisionRadius);

    addAttribute(s_enable);

    addAttribute(s_outputRotate);

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
    
    addAttribute(s_angleLimit);
    addAttribute(s_angleConeSize);    

    return MS::kSuccess;
}

visualizerDrawOverride::visualizerDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, nullptr, true) // isAlwaysDirty=true
{}

visualizerDrawOverride::~visualizerDrawOverride() {}

MHWRender::MPxDrawOverride* visualizerDrawOverride::creator(const MObject& obj)
{
    return new visualizerDrawOverride(obj);
}

MHWRender::DrawAPI visualizerDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}

bool visualizerDrawOverride::isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    return false;
}

MUserData* visualizerDrawOverride::prepareForDraw(
    const MDagPath& objPath,
    const MDagPath& cameraPath,
    const MHWRender::MFrameContext& frameContext,
    MUserData* oldData
)
{
    visualizerGeometry::VisualizerDrawData* drawData = dynamic_cast<visualizerGeometry::VisualizerDrawData*>(oldData);
    if (!drawData)
    {
        drawData = new visualizerGeometry::VisualizerDrawData();
    }
    
    MStatus status;
    const MObject node = objPath.node(&status);
    if (!status)
    {
        return drawData;
    }

    // display switches
    const bool drawCollisionRadius = visualizerGeometry::getBoolPlug(node, boneDynamicsVisualizer::s_drawCollisionRadius, false);
    const bool drawAngleLimit = visualizerGeometry::getBoolPlug(node, boneDynamicsVisualizer::s_drawAngleLimit, false);
    
    // get common data
    const bool enable = visualizerGeometry::getBoolPlug(node, boneDynamicsVisualizer::s_enable, true);
    const MVector outputRotate = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_outputRotate);

    // build bone input data
    boneDynamicsUtils::BoneInput boneInput;
    
    boneInput.boneTranslate = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_boneTranslate);
    boneInput.boneJointOrient = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_boneJointOrient);
    boneInput.boneParentMatrix = visualizerGeometry::getMatrixPlug(node, boneDynamicsVisualizer::s_boneParentMatrix);
    boneInput.boneParentInverseMatrix = visualizerGeometry::getMatrixPlug(node, boneDynamicsVisualizer::s_boneParentInverseMatrix);
    boneInput.endTranslate = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_endTranslate);
    boneInput.rotationOffset = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_rotationOffset);
    boneInput.boneScale = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_boneScale, MVector(1.0, 1.0, 1.0));
    boneInput.boneInverseScale = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_boneInverseScale, MVector(1.0, 1.0, 1.0));
    boneInput.endScale = visualizerGeometry::getVectorPlug(node, boneDynamicsVisualizer::s_endScale, MVector(1.0, 1.0, 1.0));
    boneInput.radius = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_radius);

    // build pose data
    const boneDynamicsUtils::PoseData pose = boneDynamicsUtils::buildPoseData(boneInput);
    //const double radius = pose.radius;

    // build matrices for visualization
    const MMatrix endMatrix = boneDynamicsUtils::buildSimulatedEndWorldMatrix(boneInput, outputRotate);
    //const MMatrix angleLimitMatrix = boneDynamicsUtils::buildAngleLimitMatrix(boneInput, enable);
    const MMatrix angleLimitMatrix = boneDynamicsUtils::resetMatrixScale(enable ? pose.boneInitialWorldMatrix : pose.boneInitialWorldMatrixExcludeRO);

    // angle limit params
    const double angleLimitDegrees = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_angleLimit, 60.0);
    const double angleConeSize = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_angleConeSize, 1.0);

    // -------------------------------------------------------
    // draw
    
    drawData->radiusSphereLines.clear();
    
    if (drawCollisionRadius && pose.radius > 0.0)
    {
        visualizerGeometry::appendRadiusSphere(drawData->radiusSphereLines, pose.radius, endMatrix);
    }
    
    drawData->angleLimitLines.clear();

    if (drawAngleLimit && angleLimitDegrees > 0.0)
    {
        visualizerGeometry::appendAngleLimitCone(drawData->angleLimitLines, angleLimitDegrees, angleConeSize, angleLimitMatrix);
    }

    drawData->color = MHWRender::MGeometryUtilities::wireframeColor(objPath);
    drawData->depthPriority = MHWRender::MRenderItem::sDormantWireDepthPriority;

    return drawData;
}

bool visualizerDrawOverride::hasUIDrawables() const
{
    return true;
}

void visualizerDrawOverride::addUIDrawables(
    const MDagPath& objPath,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext,
    const MUserData* data
)
{
    const visualizerGeometry::VisualizerDrawData* drawData = dynamic_cast<const visualizerGeometry::VisualizerDrawData*>(data);
if (!drawData)
    {
        return;
    }

    drawManager.beginDrawable();

    drawManager.setColor(drawData->color);
    drawManager.setDepthPriority(drawData->depthPriority);

    if (drawData->angleLimitLines.length() > 0)
    {
        drawManager.mesh(MHWRender::MUIDrawManager::kLines, drawData->angleLimitLines );
    }

    if (drawData->radiusSphereLines.length() > 0)
    {
        drawManager.mesh(MHWRender::MUIDrawManager::kLines, drawData->radiusSphereLines);
    }

    drawManager.endDrawable();
}