#include "sphereColliderNode.h"
#include "colliderGeometry.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>

MTypeId sphereColliderNode::s_id(nodeIds::sphereColliderNode);

MString sphereColliderNode::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/sphereCollider");
MString sphereColliderNode::s_drawRegistrantId("boneDynamicsNodePlugin");

MObject sphereColliderNode::s_radius;

sphereColliderNode::sphereColliderNode() {}
sphereColliderNode::~sphereColliderNode() {}

MStatus sphereColliderNode::compute(const MPlug& plug, MDataBlock& data)
{
    return MS::kSuccess;
}

bool sphereColliderNode::isBounded() const
{
    return true;
}

MBoundingBox sphereColliderNode::boundingBox() const
{
    const MObject thisNode = thisMObject();
    const double radius = colliderGeometry::getDistancePlugAsCentimeters(thisNode, s_radius, 1.0);
    return colliderGeometry::makeSphereBoundingBox(radius);
}

void* sphereColliderNode::creator()
{
    return new sphereColliderNode();
}

MStatus sphereColliderNode::initialize()
{
    MFnUnitAttribute uAttr;

    s_radius = uAttr.create("radius", "r", MFnUnitAttribute::kDistance, 1.0);
    uAttr.setKeyable(true);

    addAttribute(s_radius);

    return MS::kSuccess;
}


sphereColliderDrawOverride::sphereColliderDrawOverride(const MObject& obj) : MHWRender::MPxDrawOverride(obj, nullptr, false) {}
sphereColliderDrawOverride::~sphereColliderDrawOverride() {}

MHWRender::MPxDrawOverride* sphereColliderDrawOverride::creator(const MObject& obj)
{
    return new sphereColliderDrawOverride(obj);
}

MHWRender::DrawAPI sphereColliderDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}

bool sphereColliderDrawOverride::isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    return true;
}

MBoundingBox sphereColliderDrawOverride::boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    const double radius = getRadius(objPath);
    return colliderGeometry::makeSphereBoundingBox(radius);
}

double sphereColliderDrawOverride::getRadius(const MDagPath& objPath) const
{
    MStatus status;
    const MObject node = objPath.node(&status);

    if (!status)
    {
        return 1.0;
    }

    const double radius = colliderGeometry::getDistancePlugAsCentimeters(node, sphereColliderNode::s_radius, 1.0);

    return radius;
}

MUserData* sphereColliderDrawOverride::prepareForDraw(
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

    const double radius = getRadius(objPath);

    drawData->lineList.clear();

    colliderGeometry::appendWireSphere(drawData->lineList, radius);

    drawData->color = MHWRender::MGeometryUtilities::wireframeColor(objPath);
    drawData->depthPriority = MHWRender::MRenderItem::sDormantWireDepthPriority;

    return drawData;
}

bool sphereColliderDrawOverride::hasUIDrawables() const
{
    return true;
}

void sphereColliderDrawOverride::addUIDrawables(
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
