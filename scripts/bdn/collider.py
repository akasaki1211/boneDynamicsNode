from typing import List, Tuple, Optional, Sequence, Union

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
        indices = cmds.getAttr(f'{bone_dynamics_node}.{col_attr}', mi=True)
        if indices is None:
            continue
        for idx in indices:
            cmds.removeMultiInstance(f'{bone_dynamics_node}.{col_attr}[{idx}]', b=True)


class _ColliderConnector:
    
    def __init__(self, bone_dynamics_node: str):
        self.bone_dynamics_node: str = bone_dynamics_node
        self._update_indices()

    def _update_indices(self):
        
        def _get_next_index(attr: str) -> int:
            indices = cmds.getAttr(f'{self.bone_dynamics_node}.{attr}', mi=True)
            return max(indices) + 1 if indices else 0
        
        self.sphere_col_idx: int = _get_next_index('sphereCollider')
        self.capsule_col_idx: int = _get_next_index('capsuleCollider')
        self.capsule_col_input_idx: int = _get_next_index('capsuleColliderInput')
        self.iplane_col_idx: int = _get_next_index('infinitePlaneCollider')
        self.mesh_col_idx: int = _get_next_index('meshCollider')
    
    def connect_sphere(self, matrix_attr: str, radius_attr: str) -> None:
        cmds.connectAttr(matrix_attr, f'{self.bone_dynamics_node}.sphereCollider[{self.sphere_col_idx}].sphereColMatrix', f=True)
        cmds.connectAttr(radius_attr, f'{self.bone_dynamics_node}.sphereCollider[{self.sphere_col_idx}].sphereColRadius', f=True)
        self.sphere_col_idx += 1

    def connect_capsule_input(self, matrix_attr: str, height_attr: str, radius_a_attr: str, radius_b_attr: str) -> None:
        cmds.connectAttr(matrix_attr, f'{self.bone_dynamics_node}.capsuleColliderInput[{self.capsule_col_input_idx}].capsuleColliderMatrix', f=True)
        cmds.connectAttr(height_attr, f'{self.bone_dynamics_node}.capsuleColliderInput[{self.capsule_col_input_idx}].capsuleColliderHeight', f=True)
        cmds.connectAttr(radius_a_attr, f'{self.bone_dynamics_node}.capsuleColliderInput[{self.capsule_col_input_idx}].capsuleColliderRadiusA', f=True)
        cmds.connectAttr(radius_b_attr, f'{self.bone_dynamics_node}.capsuleColliderInput[{self.capsule_col_input_idx}].capsuleColliderRadiusB', f=True)
        self.capsule_col_input_idx += 1

    def connect_capsule(self, matrix_a_attr: str, matrix_b_attr: str, radius_a_attr: str, radius_b_attr: str) -> None:
        cmds.connectAttr(matrix_a_attr, f'{self.bone_dynamics_node}.capsuleCollider[{self.capsule_col_idx}].capsuleColMatrixA', f=True)
        cmds.connectAttr(matrix_b_attr, f'{self.bone_dynamics_node}.capsuleCollider[{self.capsule_col_idx}].capsuleColMatrixB', f=True)
        cmds.connectAttr(radius_a_attr, f'{self.bone_dynamics_node}.capsuleCollider[{self.capsule_col_idx}].capsuleColRadiusA', f=True)
        cmds.connectAttr(radius_b_attr, f'{self.bone_dynamics_node}.capsuleCollider[{self.capsule_col_idx}].capsuleColRadiusB', f=True)
        self.capsule_col_idx += 1

    def connect_infinite_plane(self, matrix_attr: str) -> None:
        cmds.connectAttr(matrix_attr, f'{self.bone_dynamics_node}.infinitePlaneCollider[{self.iplane_col_idx}].infinitePlaneColMatrix', f=True)
        self.iplane_col_idx += 1

    def connect_mesh(self, mesh_attr: str) -> None:
        cmds.connectAttr(mesh_attr, f'{self.bone_dynamics_node}.meshCollider[{self.mesh_col_idx}]', f=True)
        self.mesh_col_idx += 1


@utils.with_traceback
@utils.undo_chunk
def connect_colliders(
        bone_dynamics_node: str, 
        colliders: Union[str, Sequence[str]],
        replace: bool = False,
        *args
    ) -> None:

    if not colliders:
        return
    
    if isinstance(colliders, str):
        colliders = [colliders]
    else:
        colliders = list(colliders)

    if replace:
        disconnect_colliders(bone_dynamics_node)

    connector = _ColliderConnector(bone_dynamics_node)

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
                connector.connect_sphere(f'{col}.worldMatrix[0]', f'{col_shape}.radius')
                is_found = True

            elif col_node_type == 'capsuleColliderNode':
                connector.connect_capsule_input(f'{col}.worldMatrix[0]', f'{col_shape}.height', f'{col_shape}.radiusA', f'{col_shape}.radiusB')
                is_found = True

            elif col_node_type == 'infinitePlaneColliderNode':
                connector.connect_infinite_plane(f'{col}.worldMatrix[0]')
                is_found = True

            elif col_node_type == 'mesh':
                connector.connect_mesh(f'{col_shape}.worldMesh[0]')
                is_found = True

        if is_found:
            continue

        # expcol-type colliders
        if cmds.attributeQuery('colliderType', n=col, ex=True):
            # get expCol's collider type
            collider_type = cmds.getAttr(f'{col}.colliderType')
            
            if collider_type == 'sphere':
                connector.connect_sphere(f'{col}.worldMatrix[0]', f'{col}.radius')
                is_found = True
            
            elif collider_type in ['capsule', 'capsule2']:
                ra = ".radius" if collider_type == 'capsule' else ".radiusA"
                rb = ".radius" if collider_type == 'capsule' else ".radiusB"
                a = cmds.listConnections(f'{col}.sphereA', s=True, d=False) or []
                b = cmds.listConnections(f'{col}.sphereB', s=True, d=False) or []
                if not a or not b:
                    print(f"Skip: {col} capsule collider is missing sphereA or sphereB connection.")
                    continue
                a = a[0]
                b = b[0]
                connector.connect_capsule(f'{a}.worldMatrix[0]', f'{b}.worldMatrix[0]', f'{col}{ra}', f'{col}{rb}')
                is_found = True
            
            elif collider_type == 'infinitePlane':
                connector.connect_infinite_plane(f'{col}.worldMatrix[0]')
                is_found = True

        if not is_found:
            print(f"Skip: {col} is not a supported collider type.")
