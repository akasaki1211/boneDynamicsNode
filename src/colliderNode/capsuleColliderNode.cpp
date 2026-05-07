#include "capsuleColliderNode.h"
#include "colliderGeometry.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>

MTypeId capsuleColliderNode::s_id(nodeIds::capsuleColliderNode);

MString capsuleColliderNode::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/capsuleCollider");
MString capsuleColliderNode::s_drawRegistrantId("boneDynamicsNodePlugin");

MObject capsuleColliderNode::s_radiusA;
MObject capsuleColliderNode::s_radiusB;
MObject capsuleColliderNode::s_height;

capsuleColliderNode::capsuleColliderNode() {}
capsuleColliderNode::~capsuleColliderNode() {}

MStatus capsuleColliderNode::compute(const MPlug& plug, MDataBlock& data)
{
    return MS::kSuccess;
}

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

void* capsuleColliderNode::creator()
{
    return new capsuleColliderNode();
}

MStatus capsuleColliderNode::initialize()
{
    MFnUnitAttribute uAttr;
    
    s_radiusA = uAttr.create("radiusA", "ra", MFnUnitAttribute::kDistance, 1.0);
    uAttr.setKeyable(true);
    uAttr.setMin(0);

    s_radiusB = uAttr.create("radiusB", "rb", MFnUnitAttribute::kDistance, 1.0);
    uAttr.setKeyable(true);
    uAttr.setMin(0);

    s_height = uAttr.create("height", "h", MFnUnitAttribute::kDistance, 2.0 );
    uAttr.setKeyable(true);
    uAttr.setMin(0);

    addAttribute(s_radiusA);
    addAttribute(s_radiusB);
    addAttribute(s_height);
    
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

    drawData->lineList.clear();

    colliderGeometry::appendWireCapsule(drawData->lineList, radiusA, radiusB, height);

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