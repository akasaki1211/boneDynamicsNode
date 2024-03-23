from maya import cmds

cmds.loadPlugin("boneDynamicsNode.mll", qt=True)

def create_dynamics_node(bone, end):

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

    return boneDynamicsNode

if __name__ == "__main__":
    
    # Select in order from root to tip of the joint-chain
    joints = cmds.ls(sl=True)

    # ---------------------------------------------------

    set_name = "boneDynamicsNodeSet"
    if not cmds.objExists(set_name):
        cmds.select(cl=True)
        cmds.sets(name=set_name)

    for bone, end in zip(joints[:-1], joints[1:]):
        boneDynamicsNode = create_dynamics_node(bone, end)
        cmds.sets(boneDynamicsNode, addElement=set_name)