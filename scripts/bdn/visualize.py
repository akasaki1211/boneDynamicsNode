
from maya import cmds

from . import utils


@utils.with_traceback
@utils.undo_chunk
def create_visualizer(bone_dynamics_node: str, **kwargs):

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    dest = cmds.listConnections(bone_dynamics_node, s=False, d=True, type='boneDynamicsVisualizer')
    
    if dest:
        visualizer_shape = dest[0]
        visualizer_tm = cmds.listRelatives(visualizer_shape, parent=True, fullPath=False)[0]
    else:
        visualizer_tm, visualizer_shape = utils.create_shape_node('boneDynamicsVisualizer')
    
    utils.set_outliner_color(visualizer_tm, (0, .5, 1))
    utils.set_override_display_type(visualizer_shape, type='Reference')
    utils.set_attributes(visualizer_shape, **kwargs)

    cons = [
        ('outputRotate', 'outputRotate'),
        ('iterations', 'drawBoneCollision'),
        ('enableAngleLimit', 'drawAngleLimit'),
        ('enable', 'enable'),
        ('boneTranslate', 'boneTranslate'),
        ('boneJointOrient', 'boneJointOrient'),
        ('boneParentMatrix', 'boneParentMatrix'),
        ('boneParentInverseMatrix', 'boneParentInverseMatrix'),
        ('endTranslate', 'endTranslate'),
        ('rotationOffset', 'rotationOffset'),
        ('boneScale', 'boneScale'),
        ('boneInverseScale', 'boneInverseScale'),
        ('endScale', 'endScale'),
        ('radius', 'radius'),
        ('angleLimit', 'angleLimit'),
        ('rootRadius', 'rootRadius'),
        ('boneAsCapsule', 'boneAsCapsule')
    ]

    for src_attr, dst_attr in cons:
        cmds.connectAttr(f'{bone_dynamics_node}.{src_attr}', f'{visualizer_shape}.{dst_attr}', f=True)

    return visualizer_tm, visualizer_shape