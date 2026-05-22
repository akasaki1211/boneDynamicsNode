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
    if (!status)
    {
        status.perror("register " boneDynamicsNodeName);
        return status;
    }

    // sphereColliderNode
    status = plugin.registerNode(
        sphereColliderNodeName, 
        sphereColliderNode::s_id, 
        sphereColliderNode::creator, 
        sphereColliderNode::initialize,
        MPxNode::kLocatorNode,
        &sphereColliderNode::s_drawDbClassification
    );
    if (!status)
    {
        status.perror("register " sphereColliderNodeName);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        sphereColliderNode::s_drawDbClassification,
        sphereColliderNode::s_drawRegistrantId,
        sphereColliderDrawOverride::creator
    );
    if (!status)
    {
        status.perror("registerDrawOverrideCreator " sphereColliderNodeName);
        return status;
    }

    // capsuleColliderNode
    status = plugin.registerNode(
        capsuleColliderNodeName, 
        capsuleColliderNode::s_id, 
        capsuleColliderNode::creator, 
        capsuleColliderNode::initialize,
        MPxNode::kLocatorNode,
        &capsuleColliderNode::s_drawDbClassification
    );
    if (!status)
    {
        status.perror("register " capsuleColliderNodeName);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        capsuleColliderNode::s_drawDbClassification,
        capsuleColliderNode::s_drawRegistrantId,
        capsuleColliderDrawOverride::creator
    );
    if (!status)
    {
        status.perror("registerDrawOverrideCreator " capsuleColliderNodeName);
        return status;
    }

    // infinitePlaneColliderNode
    status = plugin.registerNode(
        infinitePlaneColliderNodeName, 
        infinitePlaneColliderNode::s_id, 
        infinitePlaneColliderNode::creator, 
        infinitePlaneColliderNode::initialize,
        MPxNode::kLocatorNode,
        &infinitePlaneColliderNode::s_drawDbClassification
    );
    if (!status)
    {
        status.perror("register " infinitePlaneColliderNodeName);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        infinitePlaneColliderNode::s_drawDbClassification,
        infinitePlaneColliderNode::s_drawRegistrantId,
        infinitePlaneColliderDrawOverride::creator
    );
    if (!status)
    {
        status.perror("registerDrawOverrideCreator " infinitePlaneColliderNodeName);
        return status;
    }

    // boneDynamicsVisualizer
    status = plugin.registerNode(
        boneDynamicsVisualizerName,
        boneDynamicsVisualizer::s_id,
        boneDynamicsVisualizer::creator,
        boneDynamicsVisualizer::initialize,
        MPxNode::kLocatorNode,
        &boneDynamicsVisualizer::s_drawDbClassification
    );
    if (!status)
    {
        status.perror("register " boneDynamicsVisualizerName);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        boneDynamicsVisualizer::s_drawDbClassification,
        boneDynamicsVisualizer::s_drawRegistrantId,
        visualizerDrawOverride::creator
    );
    if (!status)
    {
        status.perror("registerDrawOverrideCreator " boneDynamicsVisualizerName);
        return status;
    }

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

    if (!status)
    {
        status.perror("deregisterDrawOverrideCreator " boneDynamicsVisualizerName);
        return status;
    }

    status = plugin.deregisterNode(boneDynamicsVisualizer::s_id);

    if (!status)
    {
        status.perror("deregisterNode " boneDynamicsVisualizerName);
        return status;
    }

    // infinitePlaneColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        infinitePlaneColliderNode::s_drawDbClassification,
        infinitePlaneColliderNode::s_drawRegistrantId
    );

    if (!status)
    {
        status.perror("deregisterDrawOverrideCreator " infinitePlaneColliderNodeName);
        return status;
    }

    status = plugin.deregisterNode(infinitePlaneColliderNode::s_id);

    if (!status)
    {
        status.perror("deregisterNode " infinitePlaneColliderNodeName);
        return status;
    }

    // capsuleColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        capsuleColliderNode::s_drawDbClassification,
        capsuleColliderNode::s_drawRegistrantId
    );

    if (!status)
    {
        status.perror("deregisterDrawOverrideCreator " capsuleColliderNodeName);
        return status;
    }

    status = plugin.deregisterNode(capsuleColliderNode::s_id);

    if (!status)
    {
        status.perror("deregisterNode " capsuleColliderNodeName);
        return status;
    }

    // sphereColliderNode
    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        sphereColliderNode::s_drawDbClassification,
        sphereColliderNode::s_drawRegistrantId
    );

    if (!status)
    {
        status.perror("deregisterDrawOverrideCreator " sphereColliderNodeName);
        return status;
    }

    status = plugin.deregisterNode(sphereColliderNode::s_id);

    if (!status)
    {
        status.perror("deregister " sphereColliderNodeName);
        return status;
    }

    // boneDynamicsNode
    status = plugin.deregisterNode(boneDynamicsNode::s_id);
    if (!status)
    {
        status.perror("deregister " boneDynamicsNodeName);
        return status;
    }

    return MS::kSuccess;
}