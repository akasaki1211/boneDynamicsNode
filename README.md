# boneDynamicsNode

A custom node that applies bone dynamics for one section. Multiple nodes are used in a chain.  

> ⚠**Warning**⚠  
> Internal Use ID `0x7b001` is used.

## Basic Usage
Create any joint chain and connect the `boneDynamicsNode` per section. The minimum required connections are as follows:

### Required Connections

- `Time`  
- `Bone Translate`  
- `Bone Joint Orient`  
- `Bone Parent Matrix`  
- `Bone Parent Inverse Matrix`  
- `End Translate`  
- `Output Rotate`  

To rotate joint1, with joint2 as the tip...

![required_connections](.images/required_connections.png)

Let's move the current frame to 1 or later and move the root. The joint-chain shakes!  

![basic_usage](.images/basic_usage.gif)

> 💡**Requirements for Joint**  
> Joint chain must meet the requirements [here](#Requirements-for-Joint).

> 💡**Sample Script**  
> [basic_usage.py](scripts/basic_usage.py) is a script to connect bonedynamicsNode to any joint chain. Please select and execute them in order from the root of the joint to the tip of the joint.  

### Attributes to Check
- `Enable` : Turning it off disables all calculations.  
- `Reset Time` : Dynamics are disabled when the current frame is equal to or less than this value. In other words, start frame of the simulation.  
- `Fps` : Used to calculate delta time. Enter a value equal to the FPS of the scene. Default is 30.  

### Dynamics Attributes
- `Damping` : Attenuates speed. The larger the value, the harder it is to accelerate.  
- `Elasticity` : Force to return to the original posture.  
- `Stiffness` : Suppresses changes between frames (steps). Setting to 1 will result in loss of motion.  
- `Mass` : Affects the force to return to the original posture.  
- `Gravity` : If Y-up and the unit is in centimeters, it is [0,-980,0].  
- `Gravity Multiply` : Will be multiplied by Gravity.  

### Bake to Keyframes
Select the joint and execute "Bake Simulation" from the "Key" menu. Then delete the boneDynamicsNode.  

### Requirements for Joint
All of the following checks can be satisfied by creating joints as usual.  
- Rotate should be [0,0,0], with only the Joint Orient having a value.
- Do not edit rotatePivot, rotatePivotTranslate, scalePivot, or scalePivotTranslate.  
- Rotate Oeder is only available for xyz.  
- Rotate Axis should remain [0,0,0].  
- Leave Inherits Transform checked.  
- Receive InverseScale from the parent Scale and leave Segment Scale Compensate checked. Shear not supported.  

## Features
### Collisions

4 types of collisions are available: ground, sphere, capsule, and infinite plane. Colliders can be created with [maya_expressionCollision](https://github.com/akasaki1211/maya_expressionCollision).  

![collisions](.images/collisions.gif)

- `Radius` : Radius of the end-joint.  
- `Iterations` : Higher values increase the accuracy of collisions. Recommended value is 3 to 5.  
- `Enable Ground Col` : Enable ground collision.  
- `Ground Height` : Height of the ground.  
- `Sphere Col Matrix` : Connect the sphere collider worldMatrix.  
- `Sphere Col Radius` : Radius of sphere collider.  
- `Capsule Col Matrix A` `Capsule Col Matrix B` : Connect worldMatrix on one side of the capsule collider.  
- `Capsule Col Radius A` `Capsule Col Radius B` : Connect the radius of one side of the capsule collider.  
- `Infinite Plane Col Matrix` : Connect worldMatrix of infinite plane collider.  

The attributes that connect the collider are in a list, so you can use more than one. Ground Collision do not need connections.  
 
![collider_connections](.images/collider_connections.png)

> 💡**Visualize Radius**  
> Place a nurbsSphere or implicitSphere as a child of end-joint and connect `Radius`.  
> ![visualize_radius](.images/visualize_radius.png)

### Specify Target Pose

The converging pose can be manipulated by duplicating the joint-chain and connecting Rotate to `Rotation Offset`.  

![rotation_offset](.images/rotation_offset.gif)

![rotation_offset_connections](.images/rotation_offset_connections.png)

### Offset Transform

You can cancel the transform by connecting the worldMatrix of the node you do not want affected (such as the root controller of your character) to the `Offset Matrix`.

![offset_matrix](.images/offset_matrix.gif)

![offset_matrix_connections](.images/offset_matrix_connections.png)

### Supports Scale per Section
Connecting each scale value to `Bone Scale`, `Bone Inverse Scale`, and `End Scale` enables a scale per section.  

![scale](.images/scale.gif)

![scale_connections](.images/scale_connections.png)

### Branching

![branching](.images/branching.gif)

Branching is possible, but good results are obtained with joints like the one on the left of the image. The right one does not give good results.

![branching_skeleton](.images/branching_skeleton.png)

> 💡**Sample Script**  
> [advanced_usage.py](scripts/advanced_usage.py) is a script that connects a bonedynamicsNode to an arbitrary chain of joints. It is executed by selecting each joint in turn from root to tip.  
> - Enable per-section scaling.  
> - If place the collider created by expcol as a child of 'collider_grp', to be connected.  
> - If duplicate the joint-chain to be simulated and add '_target' to the end of the name, to allow manipulation of the target posture.  
> - If a node named "offset" exists, will be connected to cancel the transform.  

## Pre-built plug-ins
Pre-built `boneDynamicsNode.mll` in the [plug-ins](./plug-ins) folder. Install to the appropriate Maya version and ready to use.  
|Build|Plug-in<br>(*Click on "Download raw file" at the link.)|
|---|---|
|Maya 2022 Update 5 win64|[Download](./plug-ins/2022/boneDynamicsNode.mll)|
|Maya 2023 Update 3 win64|[Download](./plug-ins/2023/boneDynamicsNode.mll)|
|Maya 2024 Update 2 win64|[Download](./plug-ins/2024/boneDynamicsNode.mll)|

## TODO
- [ ] Angle Limit  
- [ ] Additional Force  
- [ ] Stretchable