from maya import cmds
import bdn


# Select in order from root to tip of the joint-chain
joints = cmds.ls(sl=True)


# Create a dynamics node for each bone section in the selected joint-chain.
# bdn.create_dynamics_node returns the created node.
bd_nodes = []
for bone, end in zip(joints[:-1], joints[1:]):
    bd_node = bdn.create_dynamics_node(bone, end)
    bd_nodes.append(bd_node)