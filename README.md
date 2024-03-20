# boneDynamicsNode

A custom node that applies bone dynamics for one section. Multiple nodes are used in a chain.  

> ⚠**Warning**⚠  
> Internal Use ID `0x7b001` is used.

## Basic Usage
Create any joint chain and connect the `boneDynamicsNode` per section. The minimum required connections are as follows:

> 💡Joint chain must meet the requirements [here](#Requirements-for-Joint).

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

Now move root and it will shake!

> 💡[basic_usage.py](scripts/basic_usage.py) is a script to connect bonedynamicsNode to any joint chain. Please select and execute them in order from the root of the joint to the tip of the joint.  

### Attributes to Check
- `Enable` : Turning it off disables all calculations.  
- `Reset Time` : Dynamics are disabled when the current frame is equal to or less than this value. In other words, start frame of the simulation.  
- `Fps` : Used to calculate delta time. Enter a value equal to the FPS of the scene. Default is 30.  

### Adjust Dynamics Attributes
- `Damping` : Attenuates speed. The larger the value, the harder it is to accelerate.  
- `Elasticity` : Force to return to the original posture.  
- `Stiffness` : Suppresses changes between frames (steps). Setting to 1 will result in loss of motion.  
- `Mass` : Mass. Affects the force to return to the original posture.  
- `Gravity` : If the unit is centimeter, it is -980.  
- `GravityMultiply` : The result is multiplied by Gravity.  

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

- `Radius` : Radius of the joint.  
- `Iterations` : Higher values increase the accuracy of collisions. Recommended value is 3 to 5.  
- `Enable Ground Col` : Enable ground collision.  
- `Ground Height` : Height of the ground.  
- `Sphere Col Matrix` : Connect the sphere collider worldMatrix.  
- `Sphere Col Radius` : Radius of sphere collider.  
- `Capsule Col Matrix A` `Capsule Col Matrix B` : Connect worldMatrix on one side of the capsule collider.  
- `Capsule Col Radius A` `Capsule Col Radius B` : Connect the radius of one side of the capsule collider.  
- `Infinite Plane Col Matrix` : Connect worldMatrix of infinite plane collider.  

> 💡**Visualize Radius**  
> Place a nurbsSphere or implicitSphere as a child of end-joint and set the radius value.

### Specify Target Pose

The converging pose can be manipulated by duplicating the joint-chain and connecting Rotate to `Rotation Offset`.  

### Offset Transform

You can cancel the transform by connecting the worldMatrix of the node you do not want affected (such as the root controller of your character) to the `Offset Matrix`.

### Supports Scale per Section
Connecting each scale value to `Bone Scale`, `Bone Inverse Scale`, and `End Scale` enables a scale per section.  

### Branching
Preparing explanation...

## Pre-built plug-ins
Pre-built `boneDynamicsNode.mll` in the [plug-ins](./plug-ins) folder. Install to the appropriate Maya version and ready to use.  
|Build|Plug-in<br>(*Click on "Download raw file" at the link.)|
|---|---|
|Maya 2022 Update 5 win64|preparing...|
|Maya 2023 Update 3 win64|preparing...|
|Maya 2024 Update 2 win64|[Download](./plug-ins/2024/colDetectionNode.mll)|

## TODO
- [ ] Angle Limit  
- [ ] Additional Force  
- [ ] Stretchable