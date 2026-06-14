from typing import List, Sequence, Optional, Union

from maya import cmds

from . import utils, collider, visualize


@utils.with_traceback
@utils.undo_chunk
def create_dynamics_node(
        bone: str, 
        end: str, 
        scalable: bool = False, 
        target_bone: Optional[str] = None, 
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
    
    # create boneDynamicsNode and set attributes
    bone_dynamics_node = cmds.createNode("boneDynamicsNode")
    attributes = dict(kwargs)
    if 'fps' not in attributes:
        attributes['fps'] = utils.get_fps()
    utils.set_attributes(bone_dynamics_node, **attributes)

    # basic connections
    cmds.connectAttr('time1.outTime', f'{bone_dynamics_node}.time', f=True)
    cmds.connectAttr(f'{bone}.translate', f'{bone_dynamics_node}.boneTranslate', f=True)
    cmds.connectAttr(f'{bone}.parentMatrix[0]', f'{bone_dynamics_node}.boneParentMatrix', f=True)
    cmds.connectAttr(f'{bone}.parentInverseMatrix[0]', f'{bone_dynamics_node}.boneParentInverseMatrix', f=True)
    cmds.connectAttr(f'{bone}.jointOrient', f'{bone_dynamics_node}.boneJointOrient', f=True)
    cmds.connectAttr(f'{end}.translate', f'{bone_dynamics_node}.endTranslate', f=True)
    cmds.connectAttr(f'{bone_dynamics_node}.outputRotate', f'{bone}.rotate', f=True)

    if scalable:
        cmds.connectAttr(f'{bone}.scale', f'{bone_dynamics_node}.boneScale', f=True)
        cmds.connectAttr(f'{bone}.inverseScale', f'{bone_dynamics_node}.boneInverseScale', f=True)
        cmds.connectAttr(f'{end}.scale', f'{bone_dynamics_node}.endScale', f=True)

    # additional connections
    if target_bone and cmds.objExists(target_bone):
        cmds.connectAttr(f'{target_bone}.rotate', f'{bone_dynamics_node}.rotationOffset', f=True)

    if offset_node and cmds.objExists(offset_node):
        cmds.connectAttr(f'{offset_node}.worldMatrix[0]', f'{bone_dynamics_node}.offsetMatrix', f=True)

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
            cmds.connectAttr(f'{additional_force_node}.worldMatrix[0]', f'{vp}.matrix', f=True)
        cmds.connectAttr(f'{vp}.output', f'{bone_dynamics_node}.additionalForce', f=True)
    
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
