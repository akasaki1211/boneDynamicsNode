#pragma once

#include <maya/MPxLocatorNode.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MUIDrawManager.h>

class boneDynamicsVisualizer : public MPxLocatorNode
{
public:
    boneDynamicsVisualizer();
    ~boneDynamicsVisualizer() override;

    // No compute()

    bool isBounded() const override;
    MBoundingBox boundingBox() const override;

    static void* creator();
    static MStatus initialize();

    static MTypeId s_id;
    static MString s_drawDbClassification;
    static MString s_drawRegistrantId;

    // display switches
    static MObject s_drawAngleLimit;
    static MObject s_drawCollisionRadius;

    // simulate enable
    static MObject s_enable;

    // simulated rotate
    static MObject s_outputRotate;

    // bone input
    static MObject s_boneTranslate;
    static MObject s_boneJointOrient;
    static MObject s_boneParentMatrix;
    static MObject s_boneParentInverseMatrix; // This value is not used, but it is required as an argument for buildPoseData.
    static MObject s_boneScale;
    static MObject s_boneInverseScale;
    
    // end inputs
    static MObject s_endTranslate;
    static MObject s_endScale;

    // rotation offset input
    static MObject s_rotationOffset;

    // radius input (not scaled)
    static MObject s_radius;
    
    // angle limit
    static MObject s_angleLimit;
    static MObject s_angleConeSize;
};

class visualizerDrawOverride : public MHWRender::MPxDrawOverride
{
public:
    static MHWRender::MPxDrawOverride* creator(const MObject& obj);

    ~visualizerDrawOverride() override;

    // the corresponding drawing API
    MHWRender::DrawAPI supportedDrawAPIs() const override;

    // whether to use the node's bounding box
    bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;

    // returns the bounding box of the node
    //MBoundingBox boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const override;

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
    explicit visualizerDrawOverride(const MObject& obj);
};