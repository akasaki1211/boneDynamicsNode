#include "boneDynamicsNode.h"
#include "colliderNode/sphereColliderNode.h"

#include <maya/MFnPlugin.h>
#include <maya/MDrawRegistry.h>

#define boneDynamicsNodeName "boneDynamicsNode"
#define sphereColliderNodeName "sphereColliderNode"

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

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);

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