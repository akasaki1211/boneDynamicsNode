from maya import cmds

cmds.loadPlugin("boneDynamicsNode.mll", qt=True)

def create_dynamics_node(
        bone, 
        end, 
        scalable=False, 
        target_bone=None, 
        offset_node=None, 
        colliders=[],
        visualize=True
    ):

    if not bone in cmds.listRelatives(end, p=True):
        print("Exit: {} is not {}'s parent.".format(bone, end))
        return
    
    boneDynamicsNode = cmds.createNode("boneDynamicsNode")

    cmds.connectAttr('time1.outTime', boneDynamicsNode + '.time', force=True)
    cmds.connectAttr(bone + '.translate', boneDynamicsNode + '.boneTranslate', f=True)
    cmds.connectAttr(bone + '.parentMatrix[0]', boneDynamicsNode + '.boneParentMatrix', f=True)
    cmds.connectAttr(bone + '.parentInverseMatrix[0]', boneDynamicsNode + '.boneParentInverseMatrix', f=True)
    cmds.connectAttr(bone + '.jointOrient', boneDynamicsNode + '.boneJointOrient', f=True)
    cmds.connectAttr(end + '.translate', boneDynamicsNode + '.endTranslate', f=True)

    cmds.connectAttr(boneDynamicsNode + '.outputRotate', bone + '.rotate', f=True)
    
    if scalable:
        cmds.connectAttr(bone + '.scale', boneDynamicsNode + '.boneScale', f=True)
        cmds.connectAttr(bone + '.inverseScale', boneDynamicsNode + '.boneInverseScale', f=True)
        cmds.connectAttr(end + '.scale', boneDynamicsNode + '.endScale', f=True)

    if target_bone:
        if cmds.objExists(target_bone):
            cmds.connectAttr(target_bone + '.rotate', boneDynamicsNode + '.rotationOffset', f=True)

    if offset_node:
        if cmds.objExists(offset_node):
            cmds.connectAttr(offset_node + '.worldMatrix[0]', boneDynamicsNode + '.offsetMatrix', f=True)

    if visualize:
        # angle limit
        angle_cone = cmds.createNode("implicitCone")
        angle_cone_tm = cmds.listRelatives(angle_cone, p=True)[0]
        angle_cone_ro = cmds.createNode("transform", n="{}_cone_ro".format(bone))
        angle_cone_root = cmds.createNode("transform", n="{}_cone_root".format(bone))
        cmds.setAttr(angle_cone_tm + '.ry', -90)
        cmds.parent(angle_cone_tm, angle_cone_ro, r=True)
        cmds.parent(angle_cone_ro, angle_cone_root, r=True)
        bone_parent = cmds.listRelatives(bone, p=True)
        if bone_parent:
            cmds.parent(angle_cone_root, bone_parent[0], r=True)
        cmds.connectAttr(boneDynamicsNode + '.boneTranslate', angle_cone_root + '.translate', f=True)
        cmds.connectAttr(boneDynamicsNode + '.boneJointOrient', angle_cone_root + '.rotate', f=True)
        cmds.connectAttr(boneDynamicsNode + '.rotationOffset', angle_cone_ro + '.rotate', f=True)
        cmds.connectAttr(boneDynamicsNode + '.enableAngleLimit', angle_cone_root + '.v', f=True)
        cmds.connectAttr(boneDynamicsNode + '.angleLimit', angle_cone + '.coneAngle', f=True)
        cmds.setAttr(angle_cone + '.coneCap', 2)
        cmds.setAttr(angle_cone_tm + '.overrideEnabled', 1)
        cmds.setAttr(angle_cone_tm + '.overrideDisplayType', 2)

        # collision radius
        if colliders:
            radius_sphere = cmds.createNode("implicitSphere")
            cmds.connectAttr(boneDynamicsNode + '.radius', radius_sphere + '.radius', f=True)
            radius_sphere_tm = cmds.listRelatives(radius_sphere, p=True)[0]
            cmds.parent(radius_sphere_tm, end, r=True)
            cmds.setAttr(radius_sphere_tm + '.overrideEnabled', 1)
            cmds.setAttr(radius_sphere_tm + '.overrideDisplayType', 2)
    
    sphere_col_idx = 0
    capsule_col_idx = 0
    iplane_col_cidx = 0

    for col in colliders:
        
        if not cmds.objExists(col):
            print("Skip: {} is not found.".format(col))
            continue

        if not cmds.attributeQuery('colliderType', n=col, ex=True):
            print("Skip: {} has no 'colliderType' attribute.".format(col))
            continue

        colliderType = cmds.getAttr(col + '.colliderType')
        
        if colliderType == 'sphere':
            cmds.connectAttr(col + ".worldMatrix[0]", boneDynamicsNode + ".sphereCollider[{}].sphereColMatrix".format(sphere_col_idx), f=True)
            cmds.connectAttr(col + ".radius", boneDynamicsNode + ".sphereCollider[{}].sphereColRadius".format(sphere_col_idx), f=True)
            sphere_col_idx += 1
        
        elif colliderType in ['capsule', 'capsule2']:
            radius_attr_a = ".radius" if colliderType == 'capsule' else ".radiusA"
            radius_attr_b = ".radius" if colliderType == 'capsule' else ".radiusB"
            a = cmds.listConnections(col + '.sphereA', d=0)[0]
            b = cmds.listConnections(col + '.sphereB', d=0)[0]
            cmds.connectAttr(a + ".worldMatrix[0]", boneDynamicsNode + ".capsuleCollider[{}].capsuleColMatrixA".format(capsule_col_idx), f=True)
            cmds.connectAttr(b + ".worldMatrix[0]", boneDynamicsNode + ".capsuleCollider[{}].capsuleColMatrixB".format(capsule_col_idx), f=True)
            cmds.connectAttr(col + radius_attr_a, boneDynamicsNode + ".capsuleCollider[{}].capsuleColRadiusA".format(capsule_col_idx), f=True)
            cmds.connectAttr(col + radius_attr_b, boneDynamicsNode + ".capsuleCollider[{}].capsuleColRadiusB".format(capsule_col_idx), f=True)
            capsule_col_idx += 1
        
        elif colliderType == 'infinitePlane':
            cmds.connectAttr(col + ".worldMatrix[0]", boneDynamicsNode + ".infinitePlaneCollider[{}].infinitePlaneColMatrix".format(iplane_col_cidx), f=True)
            iplane_col_cidx += 1

    return boneDynamicsNode

if __name__ == "__main__":
    
    # Select in order from root to tip of the joint-chain.
    joints = cmds.ls(sl=True)
    
    # Enable per-section scaling.
    scalable = True
    
    # Place the collider created by expcol as a child of 'collider_grp'.
    colliders = []
    if cmds.objExists("collider_grp"):
        colliders = cmds.ls(cmds.listRelatives("collider_grp", c=True), tr=True)
    
    # Duplicate the joint-chain to be simulated and add '_target' to the postfix.
    target_bone_postfix = "_target"
    
    # Name of the node to offset the transform.
    offset_node_name = "offset"
    
    # ---------------------------------------------------

    set_name = "boneDynamicsNodeSet"
    if not cmds.objExists(set_name):
        cmds.select(cl=True)
        cmds.sets(name=set_name)

    for bone, end in zip(joints[:-1], joints[1:]):
        boneDynamicsNode = create_dynamics_node(
            bone, 
            end,
            scalable=scalable, 
            target_bone=bone+target_bone_postfix, 
            offset_node=offset_node_name, 
            colliders=colliders,
            visualize=True
        )

        if boneDynamicsNode:
            cmds.sets(boneDynamicsNode, addElement=set_name)