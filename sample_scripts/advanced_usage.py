from maya import cmds
import bdn


# Select in order from root to tip of the joint-chain.
joints = cmds.ls(sl=True)


if cmds.objExists("collider_grp"):
    # If "collider_grp" exists, retrieve the child nodes.
    colliders = cmds.ls(cmds.listRelatives("collider_grp", c=True), tr=True)
else:
    # Create example sphere, capsule, and infinite-plane colliders.
    sphere_col, sphere_col_shape = bdn.create_sphere_collider(radius=3)
    capsule_col, capsule_col_shape = bdn.create_capsule_collider(height=10, radius_a=1.5, radius_b=2)
    iplane_col, iplane_col_shape = bdn.create_infinite_plane_collider()
    colliders = [sphere_col, capsule_col, iplane_col]
    cmds.select(colliders)
    cmds.group(name="collider_grp")


# Attribute values applied to every boneDynamicsNode created below.
additional_attrs = {
    "damping": 0.3,
    "elasticity": 80.0,
    "additionalForceScale": 10.0,
    "rootRadius": 1.5,
    "radius": 1.5,
    "boneAsCapsule": True,
    "enableAngleLimit": True, 
}


# Create a boneDynamicsNode for each bone section and wire in targets, colliders, and forces.
for bone, end in zip(joints[:-1], joints[1:]):
    bdn.create_dynamics_node(
        bone, 
        end, 
        scalable=True,                      # Enable per-section scaling.
        target_bone=f'{bone}_target',       # The target posture is manipulated using a joint-chain in which "{bone}_target".
        connect_target_scale=True,          # Connect scale from the target joint.
        offset_node="offset",               # Name of the node to offset the transform.
        create_visualizer=True,             # Create viewport visualizers.
        colliders=colliders,                # List of collider names.
        additional_force_node="wind",       # Node name that controls the direction of additional force
        additional_force_init_vec=(0,0,-1), # Initial direction
        set_name='boneDynamicsNodeSet',     # ObjectSet name that groups boneDynamicsNode.
        **additional_attrs
    )
