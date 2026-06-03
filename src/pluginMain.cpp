#include "boneDynamicsNode.h"
#include "colliderNode/sphereColliderNode.h"
#include "colliderNode/capsuleColliderNode.h"
#include "colliderNode/infinitePlaneColliderNode.h"
#include "visualizerNode/boneDynamicsVisualizer.h"

#include <maya/MFnPlugin.h>
#include <maya/MDrawRegistry.h>

#define boneDynamicsNodeName "boneDynamicsNode"
#define sphereColliderNodeName "sphereColliderNode"
#define capsuleColliderNodeName "capsuleColliderNode"
#define infinitePlaneColliderNodeName "infinitePlaneColliderNode"
#define boneDynamicsVisualizerName "boneDynamicsVisualizer"

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Hiroyuki Akasaki", "0.5.0-dev", "Any");

    // boneDynamicsNode
    status = plugin.registerNode(
        boneDynamicsNodeName, 
        boneDynamicsNode::s_id, 
        boneDynamicsNode::creator, 
        boneDynamicsNode::initialize
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // sphereColliderNode
    status = plugin.registerNode(
        sphereColliderNodeName, 
        sphereColliderNode::s_id, 
        sphereColliderNode::creator, 
        sphereColliderNode::initialize,
        MPxNode::kLocatorNode,
        &sphereColliderNode::s_drawDbClassification
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        sphereColliderNode::s_drawDbClassification,
        sphereColliderNode::s_drawRegistrantId,
        sphereColliderDrawOverride::creator
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // capsuleColliderNode
    status = plugin.registerNode(
        capsuleColliderNodeName, 
        capsuleColliderNode::s_id, 
        capsuleColliderNode::creator, 
        capsuleColliderNode::initialize,
        MPxNode::kLocatorNode,
        &capsuleColliderNode::s_drawDbClassification
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        capsuleColliderNode::s_drawDbClassification,
        capsuleColliderNode::s_drawRegistrantId,
        capsuleColliderDrawOverride::creator
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // infinitePlaneColliderNode
    status = plugin.registerNode(
        infinitePlaneColliderNodeName, 
        infinitePlaneColliderNode::s_id, 
        infinitePlaneColliderNode::creator, 
        infinitePlaneColliderNode::initialize,
        MPxNode::kLocatorNode,
        &infinitePlaneColliderNode::s_drawDbClassification
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        infinitePlaneColliderNode::s_drawDbClassification,
        infinitePlaneColliderNode::s_drawRegistrantId,
        infinitePlaneColliderDrawOverride::creator
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // boneDynamicsVisualizer
    status = plugin.registerNode(
        boneDynamicsVisualizerName,
        boneDynamicsVisualizer::s_id,
        boneDynamicsVisualizer::creator,
        boneDynamicsVisualizer::initialize,
        MPxNode::kLocatorNode,
        &boneDynamicsVisualizer::s_drawDbClassification
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        boneDynamicsVisualizer::s_drawDbClassification,
        boneDynamicsVisualizer::s_drawRegistrantId,
        visualizerDrawOverride::creator
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);

    // boneDynamicsVisualizer
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        boneDynamicsVisualizer::s_drawDbClassification,
        boneDynamicsVisualizer::s_drawRegistrantId
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(boneDynamicsVisualizer::s_id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // infinitePlaneColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        infinitePlaneColliderNode::s_drawDbClassification,
        infinitePlaneColliderNode::s_drawRegistrantId
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(infinitePlaneColliderNode::s_id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // capsuleColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        capsuleColliderNode::s_drawDbClassification,
        capsuleColliderNode::s_drawRegistrantId
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(capsuleColliderNode::s_id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // sphereColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        sphereColliderNode::s_drawDbClassification,
        sphereColliderNode::s_drawRegistrantId
    );
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(sphereColliderNode::s_id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // boneDynamicsNode
    status = plugin.deregisterNode(boneDynamicsNode::s_id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}