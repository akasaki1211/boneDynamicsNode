#include "capsuleColliderNode.h"
#include "colliderGeometry.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>
#include <maya/MViewport2Renderer.h>

MTypeId capsuleColliderNode::s_id(nodeIds::capsuleColliderNode);

MString capsuleColliderNode::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/capsuleCollider");
MString capsuleColliderNode::s_drawRegistrantId("boneDynamicsNodePlugin");

MObject capsuleColliderNode::s_radiusA;
MObject capsuleColliderNode::s_radiusB;
MObject capsuleColliderNode::s_height;

MObject capsuleColliderNode::s_segments;

capsuleColliderNode::capsuleColliderNode() {}
capsuleColliderNode::~capsuleColliderNode() {}

bool capsuleColliderNode::isBounded() const
{
    return true;
}

MBoundingBox capsuleColliderNode::boundingBox() const
{
    const MObject thisNode = thisMObject();
    const double radiusA = colliderGeometry::getDistancePlugAsCentimeters(thisNode, s_radiusA, 1.0);
    const double radiusB = colliderGeometry::getDistancePlugAsCentimeters(thisNode, s_radiusB, 1.0);
    const double height = colliderGeometry::getDistancePlugAsCentimeters(thisNode, s_height, 2.0);
    return colliderGeometry::makeCapsuleBoundingBox(radiusA, radiusB, height);
}

MStatus capsuleColliderNode::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray)
{
    if (
        plug == s_radiusA ||
        plug == s_radiusB ||
        plug == s_height
    )
    {
        MHWRender::MRenderer::setGeometryDrawDirty(thisMObject(), false);
    }

    return MS::kSuccess;
}

void* capsuleColliderNode::creator()
{
    return new capsuleColliderNode();
}

MStatus capsuleColliderNode::initialize()
{
    MFnUnitAttribute uAttr;
    MFnNumericAttribute nAttr;
    
    s_radiusA = uAttr.create("radiusA", "ra", MFnUnitAttribute::kDistance, 1.0);
    uAttr.setKeyable(false);
    uAttr.setChannelBox(true);
    uAttr.setMin(0.001);
    uAttr.setAffectsAppearance(true);

    s_radiusB = uAttr.create("radiusB", "rb", MFnUnitAttribute::kDistance, 1.0);
    uAttr.setKeyable(false);
    uAttr.setChannelBox(true);
    uAttr.setMin(0.001);
    uAttr.setAffectsAppearance(true);

    s_height = uAttr.create("height", "h", MFnUnitAttribute::kDistance, 2.0 );
    uAttr.setKeyable(false);
    uAttr.setChannelBox(true);
    uAttr.setMin(0.001);
    uAttr.setAffectsAppearance(true);

    s_segments = nAttr.create("segments", "s", MFnNumericData::kInt, 4);
    nAttr.setKeyable(false);
    nAttr.setChannelBox(true);
    nAttr.setMin(1);
    nAttr.setMax(16);
    nAttr.setAffectsAppearance(true);

    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_radiusA));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_radiusB));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_height));
    CHECK_MSTATUS_AND_RETURN_IT(addAttribute(s_segments));

    return MS::kSuccess;
}

capsuleColliderDrawOverride::capsuleColliderDrawOverride(const MObject& obj) : MHWRender::MPxDrawOverride(obj, nullptr, false) {}
capsuleColliderDrawOverride::~capsuleColliderDrawOverride() {}

MHWRender::MPxDrawOverride* capsuleColliderDrawOverride::creator(const MObject& obj)
{
    return new capsuleColliderDrawOverride(obj);
}

MHWRender::DrawAPI capsuleColliderDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}

bool capsuleColliderDrawOverride::isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    return true;
}

MBoundingBox capsuleColliderDrawOverride::boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    double radiusA = 1.0;
    double radiusB = 1.0;
    double height = 2.0;
    getCapsuleParameters(objPath, radiusA, radiusB, height);
    return colliderGeometry::makeCapsuleBoundingBox(radiusA, radiusB, height);
}

void capsuleColliderDrawOverride::getCapsuleParameters(const MDagPath& objPath, double& radiusA, double& radiusB, double& height) const
{
    MStatus status;
    const MObject node = objPath.node(&status);
    if (!status)
    {
        return;
    }

    radiusA = colliderGeometry::getDistancePlugAsCentimeters(node, capsuleColliderNode::s_radiusA, 1.0);
    radiusB = colliderGeometry::getDistancePlugAsCentimeters(node, capsuleColliderNode::s_radiusB, 1.0);
    height = colliderGeometry::getDistancePlugAsCentimeters(node, capsuleColliderNode::s_height, 2.0);
}

int capsuleColliderDrawOverride::getSegments(const MDagPath& objPath) const
{
    MStatus status;
    const MObject node = objPath.node(&status);
    if (!status)
    {
        return 4;
    }

    const int segments = colliderGeometry::getSegments(node, capsuleColliderNode::s_segments, 4);

    return segments;
}

MUserData* capsuleColliderDrawOverride::prepareForDraw(
    const MDagPath& objPath,
    const MDagPath& cameraPath,
    const MHWRender::MFrameContext& frameContext,
    MUserData* oldData
)
{
    colliderGeometry::ColliderDrawData* drawData = dynamic_cast<colliderGeometry::ColliderDrawData*>(oldData);
    if (!drawData)
    {
        drawData = new colliderGeometry::ColliderDrawData();
    }

    double radiusA = 1.0;
    double radiusB = 1.0;
    double height = 2.0;
    getCapsuleParameters(objPath, radiusA, radiusB, height);
    const int segments = getSegments(objPath) * 4;

    drawData->lineList.clear();

    colliderGeometry::appendWireCapsule(drawData->lineList, radiusA, radiusB, height, segments);

    drawData->color = MHWRender::MGeometryUtilities::wireframeColor(objPath);
    drawData->depthPriority = MHWRender::MRenderItem::sDormantWireDepthPriority;

    return drawData;
}

bool capsuleColliderDrawOverride::hasUIDrawables() const
{
    return true;
}

void capsuleColliderDrawOverride::addUIDrawables(
    const MDagPath& objPath,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext,
    const MUserData* data
)
{
    const colliderGeometry::ColliderDrawData* drawData = dynamic_cast<const colliderGeometry::ColliderDrawData*>(data);
    if (!drawData)
    {
        return;
    }

    drawManager.beginDrawable();

    drawManager.setColor(drawData->color);
    drawManager.setDepthPriority(drawData->depthPriority);

    drawManager.mesh(MHWRender::MUIDrawManager::kLines, drawData->lineList);

    drawManager.endDrawable();
}