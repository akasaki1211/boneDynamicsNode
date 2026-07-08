from typing import List, Sequence, Optional, Union

from maya import cmds

from . import utils, collider, visualize


def _find_connected_dynamics_node(bone: str, *args) -> Optional[str]:
    connected_nodes = []
    for attr in ['rotate', 'rotateX', 'rotateY', 'rotateZ']:
        nodes = cmds.listConnections(f'{bone}.{attr}', s=True, d=False, type='boneDynamicsNode') or []
        for node in nodes:
            if node not in connected_nodes:
                connected_nodes.append(node)
    return connected_nodes[0] if connected_nodes else None


@utils.with_traceback
@utils.undo_chunk
def create_dynamics_node(
        bone: str, 
        end: str, 
        scalable: bool = False, 
        target_bone: Optional[str] = None, 
        connect_target_scale: Optional[bool] = False,
        offset_node: Optional[str] = None, 
        create_visualizer: bool = False,
        colliders: Optional[Union[str, Sequence[str]]] = None,
        additional_force_node: Optional[str] = None,
        additional_force_init_vec: Sequence[float] = (0, 0, -1),
        set_name: str = 'boneDynamicsNodeSet',
        **kwargs
    ) -> str:
    """Create and connect a boneDynamicsNode for a joint section.

    Args:
        bone: Joint driven by the created dynamics node.
        end: Tip joint that defines the end of the simulated section.
        scalable: Connect scale attributes when the joint should per-section scaling.
        target_bone: Optional node whose drives the rotation offset.
        connect_target_scale: Connect the scale from target_bone to bone.
            If the target pose is controlled using a duplicate joint, it is convenient 
            to connect the scale from the duplicate joint as well.
        offset_node: Optional node whose offsets the simulation space.
        create_visualizer: Create and connect a visualizer.
        colliders: Collider transform name, or sequence of names.
        additional_force_node: Optional node whose world matrix controls additional force direction.
        additional_force_init_vec: Initial vector that controlled by additional_force_node.
        set_name: Object set that store the created dynamics node.
        **kwargs: Attribute values to set on the created boneDynamicsNode.

    Returns:
        The created boneDynamicsNode name.
    """

    # load plugin
    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")
    
    # validate input
    if cmds.nodeType(bone) != 'joint' or cmds.nodeType(end) != 'joint':
        raise ValueError("Exit: Both bone and end must be joints.")

    # check parent-child relationship
    bone_full_paths = cmds.ls(bone, l=True) or []
    bone_path = bone_full_paths[0] if bone_full_paths else bone
    end_parents = cmds.listRelatives(end, p=True, f=True)
    
    if not end_parents:
        raise ValueError(f"Exit: {end} has no parent.")
    
    if bone_path not in end_parents:
        raise ValueError(f"Exit: {bone} is not {end}'s parent.")
    
    # create or find existing boneDynamicsNode
    bone_dynamics_node = _find_connected_dynamics_node(bone)
    if bone_dynamics_node is None:
        bone_dynamics_node = cmds.createNode("boneDynamicsNode")

    # set attributes
    attributes = dict(kwargs)
    if 'fps' not in attributes:
        attributes['fps'] = utils.get_fps()
    utils.set_attributes(bone_dynamics_node, **attributes)

    # basic connections
    utils.connect_attr('time1.outTime', f'{bone_dynamics_node}.time')
    utils.connect_attr(f'{bone}.translate', f'{bone_dynamics_node}.boneTranslate')
    utils.connect_attr(f'{bone}.parentMatrix[0]', f'{bone_dynamics_node}.boneParentMatrix')
    utils.connect_attr(f'{bone}.parentInverseMatrix[0]', f'{bone_dynamics_node}.boneParentInverseMatrix')
    utils.connect_attr(f'{bone}.jointOrient', f'{bone_dynamics_node}.boneJointOrient')
    utils.connect_attr(f'{end}.translate', f'{bone_dynamics_node}.endTranslate')
    utils.connect_attr(f'{bone_dynamics_node}.outputRotate', f'{bone}.rotate')

    if scalable:
        utils.connect_attr(f'{bone}.scale', f'{bone_dynamics_node}.boneScale')
        utils.connect_attr(f'{bone}.inverseScale', f'{bone_dynamics_node}.boneInverseScale')
        utils.connect_attr(f'{end}.scale', f'{bone_dynamics_node}.endScale')

    # additional connections
    if target_bone and cmds.objExists(target_bone):
        utils.connect_attr(f'{target_bone}.rotate', f'{bone_dynamics_node}.rotationOffset')
        if connect_target_scale:
            utils.connect_attr(f'{target_bone}.scale', f'{bone}.scale')

    if offset_node and cmds.objExists(offset_node):
        utils.connect_attr(f'{offset_node}.worldMatrix[0]', f'{bone_dynamics_node}.offsetMatrix')

    if additional_force_node and cmds.objExists(additional_force_node):
        vp = cmds.listConnections(f'{additional_force_node}.worldMatrix[0]', s=False, d=True, type='vectorProduct')
        if vp:
            vp = vp[0]
        else:
            vp = cmds.createNode('vectorProduct')
            attrs = {
                "operation": 3,
                "input1": additional_force_init_vec,
                "normalizeOutput": 1,
            }
            utils.set_attributes(vp, **attrs)
            utils.connect_attr(f'{additional_force_node}.worldMatrix[0]', f'{vp}.matrix')
        utils.connect_attr(f'{vp}.output', f'{bone_dynamics_node}.additionalForce')
    
    # visualizer
    if create_visualizer:
        visualize.create_visualizer(bone_dynamics_node)

    # colliders
    if colliders is not None:
        if isinstance(colliders, str):
            colliders = [colliders]
        else:
            colliders = list(colliders)
        collider.connect_colliders(bone_dynamics_node, colliders, replace=True)

    # add to object set
    if set_name:
        utils.add_object_to_set(bone_dynamics_node, set_name)
    
    return bone_dynamics_node
