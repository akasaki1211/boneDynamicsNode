#include "infinitePlaneColliderNode.h"
#include "colliderGeometry.h"
#include "../nodeIds.h"

#include <maya/MFnUnitAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MDrawContext.h>
#include <maya/MViewport2Renderer.h>

MTypeId infinitePlaneColliderNode::s_id(nodeIds::infinitePlaneColliderNode);

MString infinitePlaneColliderNode::s_drawDbClassification("drawdb/geometry/boneDynamicsNode/infinitePlaneCollider");
MString infinitePlaneColliderNode::s_drawRegistrantId("boneDynamicsNodePlugin");

infinitePlaneColliderNode::infinitePlaneColliderNode() {}
infinitePlaneColliderNode::~infinitePlaneColliderNode() {}

bool infinitePlaneColliderNode::isBounded() const
{
    return true;
}

MBoundingBox infinitePlaneColliderNode::boundingBox() const
{
    const MObject thisNode = thisMObject();
    return colliderGeometry::makePlaneBoundingBox();
}

/*MStatus infinitePlaneColliderNode::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray)
{
    return MS::kSuccess;
}*/

void* infinitePlaneColliderNode::creator()
{
    return new infinitePlaneColliderNode();
}

MStatus infinitePlaneColliderNode::initialize()
{
    return MS::kSuccess;
}


infinitePlaneColliderDrawOverride::infinitePlaneColliderDrawOverride(const MObject& obj) : MHWRender::MPxDrawOverride(obj, nullptr, false) {}
infinitePlaneColliderDrawOverride::~infinitePlaneColliderDrawOverride() {}

MHWRender::MPxDrawOverride* infinitePlaneColliderDrawOverride::creator(const MObject& obj)
{
    return new infinitePlaneColliderDrawOverride(obj);
}

MHWRender::DrawAPI infinitePlaneColliderDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}

bool infinitePlaneColliderDrawOverride::isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    return true;
}

MBoundingBox infinitePlaneColliderDrawOverride::boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    return colliderGeometry::makePlaneBoundingBox();
}

MUserData* infinitePlaneColliderDrawOverride::prepareForDraw(
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

    drawData->lineList.clear();

    colliderGeometry::appendWirePlane(drawData->lineList);

    drawData->color = MHWRender::MGeometryUtilities::wireframeColor(objPath);
    drawData->depthPriority = MHWRender::MRenderItem::sDormantWireDepthPriority;

    return drawData;
}

bool infinitePlaneColliderDrawOverride::hasUIDrawables() const
{
    return true;
}

void infinitePlaneColliderDrawOverride::addUIDrawables(
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
