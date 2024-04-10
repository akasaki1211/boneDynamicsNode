#include "boneDynamicsNode.h"

#include <maya/MFnPlugin.h>

#define boneDynamicsNodeName "boneDynamicsNode"

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Hiroyuki Akasaki", "0.2.1", "Any");

    status = plugin.registerNode(boneDynamicsNodeName, boneDynamicsNode::s_id, boneDynamicsNode::creator, boneDynamicsNode::initialize);
    if (!status) {
        status.perror("registerNode");
        return status;
    }

    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj);

    status = plugin.deregisterNode(boneDynamicsNode::s_id);
    if (!status) {
        status.perror("deregisterNode");
        return status;
    }

    return status;
}