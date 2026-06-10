from typing import List, Sequence, Optional

from maya import cmds

from . import utils, collider, visualize

__all__ = [
    'create_dynamics_node'
]


@utils.with_traceback
@utils.undo_chunk
def create_dynamics_node(
        bone: str, 
        end: str, 
        scalable: bool = False, 
        target_bone: Optional[str] = None, 
        offset_node: Optional[str] = None, 
        create_visualizer: bool = False,
        colliders: Optional[List[str]] = None,
        additional_force_node: Optional[str] = None,
        additional_force_init_vec: Sequence[float] = [0, 0, -1],
        set_name: str = 'boneDynamicsNodeSet',
        **kwargs
    ) -> str:

    # load plugin
    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")
    
    # validate input
    if cmds.nodeType(bone) != 'joint' or cmds.nodeType(end) != 'joint':
        raise ValueError("Exit: Both bone and end must be joints.")

    # check parent-child relationship
    end_parents = cmds.listRelatives(end, p=True)
    
    if not end_parents:
        raise ValueError(f"Exit: {end} has no parent.")
    
    if not bone in end_parents: # TODO: Handle a fullpath being provided
        raise ValueError(f"Exit: {bone} is not {end}'s parent.")
    
    # create boneDynamicsNode and set attributes
    bone_dynamics_node = cmds.createNode("boneDynamicsNode")
    utils.set_attributes(bone_dynamics_node, fps=utils.get_fps(), **kwargs)

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
            #cmds.setAttr(f'{vp}.operation', 3)
            #cmds.setAttr(f'{vp}.input1', additional_force_init_vec[0], additional_force_init_vec[1], additional_force_init_vec[2], type='double3')
            #cmds.setAttr(f'{vp}.normalizeOutput', 1)
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
        collider.connect_colliders(bone_dynamics_node, colliders, replace=True)

    # add to object set
    if set_name:
        utils.add_object_to_set(bone_dynamics_node, set_name)
    
    return bone_dynamics_node