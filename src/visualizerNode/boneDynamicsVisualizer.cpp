#include "boneDynamicsVisualizer.h"
#include "visualizerGeometry.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>


MTypeId boneDynamicsVisualizer::s_id(nodeIds::boneDynamicsVisualizer);

MString boneDynamicsVisualizer::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/visualizer");

MString boneDynamicsVisualizer::s_drawRegistrantId("boneDynamicsNodePlugin");

MObject boneDynamicsVisualizer::s_endMatrix;
MObject boneDynamicsVisualizer::s_radius;

MObject boneDynamicsVisualizer::s_angleLimitMatrix;
MObject boneDynamicsVisualizer::s_angleLimit;
MObject boneDynamicsVisualizer::s_angleConeSize;

MObject boneDynamicsVisualizer::s_drawAngleLimit;
MObject boneDynamicsVisualizer::s_drawCollisionRadius;

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

    s_endMatrix = mAttr.create("endMatrix", "emtx");
    mAttr.setKeyable(true);
    mAttr.setAffectsAppearance(true);

    s_radius = nAttr.create("radius", "r", MFnNumericData::kDouble, 1.0); // TODO: Change to MFnUnitAttribute::kDistance
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setAffectsAppearance(true);

    s_angleLimitMatrix = mAttr.create("angleLimitMatrix", "bimtx");
    mAttr.setKeyable(true);
    mAttr.setAffectsAppearance(true);

    s_angleLimit = nAttr.create("angleLimit", "al", MFnNumericData::kDouble, 60.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setMax(360);
    nAttr.setAffectsAppearance(true);

    s_angleConeSize = nAttr.create("angleConeSize", "acs", MFnNumericData::kDouble, 1.0);
    nAttr.setKeyable(true);
    nAttr.setMin(0);
    nAttr.setAffectsAppearance(true);

    s_drawAngleLimit = nAttr.create("drawAngleLimit", "dal", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    s_drawCollisionRadius = nAttr.create("drawCollisionRadius", "dcr", MFnNumericData::kBoolean, true);
    nAttr.setKeyable(true);
    nAttr.setAffectsAppearance(true);

    addAttribute(s_endMatrix);
    addAttribute(s_radius);
    
    addAttribute(s_angleLimitMatrix);
    addAttribute(s_angleLimit);
    addAttribute(s_angleConeSize);    

    addAttribute(s_drawAngleLimit);
    addAttribute(s_drawCollisionRadius);

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

    // radius sphere
    const bool drawCollisionRadius = visualizerGeometry::getBoolPlug(node, boneDynamicsVisualizer::s_drawCollisionRadius, false);
    const double radius = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_radius, 0.0);
    const MMatrix endMatrix = visualizerGeometry::getMatrixPlug(node, boneDynamicsVisualizer::s_endMatrix, MMatrix::identity);
    

    drawData->radiusSphereLines.clear();
    
    if (drawCollisionRadius && radius > 0.0)
    {
        visualizerGeometry::appendRadiusSphere(drawData->radiusSphereLines, radius, endMatrix);
    }

    // angle limit cone
    const bool drawAngleLimit = visualizerGeometry::getBoolPlug(node, boneDynamicsVisualizer::s_drawAngleLimit, false);
    const double angleLimitDegrees = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_angleLimit, 60.0);
    const MMatrix angleLimitMatrix = visualizerGeometry::getMatrixPlug(node, boneDynamicsVisualizer::s_angleLimitMatrix, MMatrix::identity);
    const double angleConeSize = visualizerGeometry::getDoublePlug(node, boneDynamicsVisualizer::s_angleConeSize, 1.0);

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