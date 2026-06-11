from typing import List, Tuple, Optional

from maya import cmds

from . import utils


@utils.with_traceback
@utils.undo_chunk
def create_sphere_collider(radius: float = 1.0, *args, **kwargs) -> Tuple[str, str]:
    
    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")
    
    transform, shape = utils.create_shape_node('sphereColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)
    cmds.setAttr(f"{shape}.radius", radius)

    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def create_capsule_collider(height: float = 1.0, radius_a: float = 1.0, radius_b: float = 1.0, *args, **kwargs) -> Tuple[str, str]:

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")
    
    transform, shape = utils.create_shape_node('capsuleColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)
    cmds.setAttr(f"{shape}.height", height)
    cmds.setAttr(f"{shape}.radiusA", radius_a)
    cmds.setAttr(f"{shape}.radiusB", radius_b)

    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def create_infinite_plane_collider(*args, **kwargs) -> Tuple[str, str]:

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")
    
    transform, shape = utils.create_shape_node('infinitePlaneColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)
    
    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def disconnect_colliders(
        bone_dynamics_node: str, 
        collider_attributes: Optional[List[str]] = None, 
        *args
    ) -> None:

    if collider_attributes is None:
        collider_attributes = [
            'sphereCollider', 
            'capsuleCollider', 
            'capsuleColliderInput', 
            'infinitePlaneCollider', 
            'meshCollider'
        ]
    
    for col_attr in collider_attributes:
        indexes = cmds.getAttr(f'{bone_dynamics_node}.{col_attr}', mi=True)
        if indexes is None:
            continue
        for idx in indexes:
            cmds.removeMultiInstance(f'{bone_dynamics_node}.{col_attr}[{idx}]', b=True)


@utils.with_traceback
@utils.undo_chunk
def connect_colliders(
        bone_dynamics_node: str, 
        colliders: List[str], 
        replace: bool = False,
        *args
    ) -> None:

    if not colliders:
        return
    
    if not isinstance(colliders, list):
        return

    if replace:
        disconnect_colliders(bone_dynamics_node)
    
    # define indexes
    def get_next_index(node: str, attr: str, *args):
        indexes = cmds.getAttr(f'{node}.{attr}', mi=True)
        return max(indexes) + 1 if indexes else 0
    
    sphere_col_idx = get_next_index(bone_dynamics_node, 'sphereCollider')
    capsule_col_idx = get_next_index(bone_dynamics_node, 'capsuleCollider')
    capsule_col_input_idx = get_next_index(bone_dynamics_node, 'capsuleColliderInput')
    iplane_col_idx = get_next_index(bone_dynamics_node, 'infinitePlaneCollider')
    mesh_col_idx = get_next_index(bone_dynamics_node, 'meshCollider')

    # collider loop
    for col in colliders:
        
        if not cmds.objExists(col):
            print(f"Skip: {col} is not found.")
            continue

        is_found = False

        col_shape = cmds.listRelatives(col, s=True, f=True)

        if col_shape:
            col_shape = col_shape[0]
            col_node_type = cmds.nodeType(col_shape)
            
            if col_node_type == 'sphereColliderNode':
                cmds.connectAttr(f'{col}.worldMatrix[0]', f'{bone_dynamics_node}.sphereCollider[{sphere_col_idx}].sphereColMatrix', f=True)
                cmds.connectAttr(f'{col_shape}.radius', f'{bone_dynamics_node}.sphereCollider[{sphere_col_idx}].sphereColRadius', f=True)
                sphere_col_idx += 1
                is_found = True

            elif col_node_type == 'capsuleColliderNode':
                cmds.connectAttr(f'{col}.worldMatrix[0]', f'{bone_dynamics_node}.capsuleColliderInput[{capsule_col_input_idx}].capsuleColliderMatrix', f=True)
                cmds.connectAttr(f'{col_shape}.height', f'{bone_dynamics_node}.capsuleColliderInput[{capsule_col_input_idx}].capsuleColliderHeight', f=True)
                cmds.connectAttr(f'{col_shape}.radiusA', f'{bone_dynamics_node}.capsuleColliderInput[{capsule_col_input_idx}].capsuleColliderRadiusA', f=True)
                cmds.connectAttr(f'{col_shape}.radiusB', f'{bone_dynamics_node}.capsuleColliderInput[{capsule_col_input_idx}].capsuleColliderRadiusB', f=True)
                capsule_col_input_idx += 1
                is_found = True

            elif col_node_type == 'infinitePlaneColliderNode':
                cmds.connectAttr(f'{col}.worldMatrix[0]', f'{bone_dynamics_node}.infinitePlaneCollider[{iplane_col_idx}].infinitePlaneColMatrix', f=True)
                iplane_col_idx += 1
                is_found = True

            # mesh collider
            elif col_node_type == 'mesh':
                cmds.connectAttr(f'{col_shape}.worldMesh[0]', f'{bone_dynamics_node}.meshCollider[{mesh_col_idx}]', f=True)
                mesh_col_idx += 1
                is_found = True

        if is_found:
            continue

        # expcol-type colliders
        if cmds.attributeQuery('colliderType', n=col, ex=True):
            # get expCol's collider type
            collider_type = cmds.getAttr(f'{col}.colliderType')
            
            if collider_type == 'sphere':
                cmds.connectAttr(f'{col}.worldMatrix[0]', f'{bone_dynamics_node}.sphereCollider[{sphere_col_idx}].sphereColMatrix', f=True)
                cmds.connectAttr(f'{col}.radius', f'{bone_dynamics_node}.sphereCollider[{sphere_col_idx}].sphereColRadius', f=True)
                sphere_col_idx += 1
                is_found = True
            
            elif collider_type in ['capsule', 'capsule2']:
                ra = ".radius" if collider_type == 'capsule' else ".radiusA"
                rb = ".radius" if collider_type == 'capsule' else ".radiusB"
                a = cmds.listConnections(f'{col}.sphereA', s=True, d=False) or []
                b = cmds.listConnections(f'{col}.sphereB', s=True, d=False) or []
                if not a or not b:
                    continue
                a = a[0]
                b = b[0]
                cmds.connectAttr(f'{a}.worldMatrix[0]', f'{bone_dynamics_node}.capsuleCollider[{capsule_col_idx}].capsuleColMatrixA', f=True)
                cmds.connectAttr(f'{b}.worldMatrix[0]', f'{bone_dynamics_node}.capsuleCollider[{capsule_col_idx}].capsuleColMatrixB', f=True)
                cmds.connectAttr(f'{col}{ra}', f'{bone_dynamics_node}.capsuleCollider[{capsule_col_idx}].capsuleColRadiusA', f=True)
                cmds.connectAttr(f'{col}{rb}', f'{bone_dynamics_node}.capsuleCollider[{capsule_col_idx}].capsuleColRadiusB', f=True)
                capsule_col_idx += 1
                is_found = True
            
            elif collider_type == 'infinitePlane':
                cmds.connectAttr(f'{col}.worldMatrix[0]', f'{bone_dynamics_node}.infinitePlaneCollider[{iplane_col_idx}].infinitePlaneColMatrix', f=True)
                iplane_col_idx += 1
                is_found = True

        if not is_found:
            print(f"Skip: {col} is not a supported collider type.")
