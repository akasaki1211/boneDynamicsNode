#pragma once

#include <maya/MPxLocatorNode.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MUIDrawManager.h>
#include <maya/MPlugArray.h>

class capsuleColliderNode : public MPxLocatorNode
{
public:
    capsuleColliderNode();
    ~capsuleColliderNode() override;

    // no compute

    bool isBounded() const override;
    MBoundingBox boundingBox() const override;

    // Explicitly notify Viewport 2.0 when inputs related to drawing become dirty
    MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;

    static void* creator();
    static MStatus initialize();

    static MTypeId s_id;
    static MString s_drawDbClassification;
    static MString s_drawRegistrantId;

    static MObject s_radiusA;
    static MObject s_radiusB;
    static MObject s_height;

    static MObject s_segments; // Use it multiply by 4
};

class capsuleColliderDrawOverride : public MHWRender::MPxDrawOverride
{
public:
    static MHWRender::MPxDrawOverride* creator(const MObject& obj);

    virtual ~capsuleColliderDrawOverride() override;

    // the corresponding drawing API
    MHWRender::DrawAPI supportedDrawAPIs() const override;

    // whether to use the node's bounding box
    bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;

    // returns the bounding box of the node
    MBoundingBox boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const override;

    // Called by Maya each time the object needs to be drawn.
    MUserData* prepareForDraw(
        const MDagPath& objPath,
        const MDagPath& cameraPath,
        const MHWRender::MFrameContext& frameContext,
        MUserData* oldData
    ) override;

    // addUIDrawables() provides access to the MUIDrawManager, which can be used
    // to queue up operations for drawing simple UI elements such as lines, circles and
    // text. To enable addUIDrawables(), override hasUIDrawables() and make it return true.
    bool hasUIDrawables() const override;
    void addUIDrawables(
        const MDagPath& objPath,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext,
        const MUserData* data
    ) override;

private:
    explicit capsuleColliderDrawOverride(const MObject& obj);
    void getCapsuleParameters(const MDagPath& objPath, double& radiusA, double& radiusB, double& height) const;
    int getSegments(const MDagPath& objPath) const;
};