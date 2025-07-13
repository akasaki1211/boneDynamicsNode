# boneDynamicsNode

![Maya](https://img.shields.io/static/v1?message=Maya&color=0696D7&logo=Autodesk&logoColor=white&label=) ![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)

**boneDynamicsNode** is a custom node for Autodesk Maya that enables dynamic simulation on joint chains. It is designed for easy integration into rigs, this node supports features such as collision handling, angle limits, target pose manipulation, and more.

> ⚠ **Warning**: Internal Use ID `0x7b001` is used.

## ✨ Features

- Simple integration with standard Maya joints
- Multiple collision types supported (sphere, capsule, infinite plane, ground, mesh)
- Angle limitation
- Specify target pose
- Cancel transforms of specific nodes
- Per-section scaling capability
- Turbulent wind
- Support for branching structures

## 🔧 Quick Installation

1. Download the appropriate `boneDynamicsNode.mll` for the Maya version from the [Releases page](https://github.com/akasaki1211/boneDynamicsNode/releases) or from [Pre-built plug-ins](#-pre-built-plug-ins).
2. Copy it to `C:\Users\<USERNAME>\Documents\maya\<MAYAVERSION>\plug-ins`.
3. Load `boneDynamicsNode.mll` using Maya's Plug-in Manager.

For additional instructions, refer to the [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation) wiki page.

## 🚀 Try it out

Please try running the following script. It creates a single-section joint with bone dynamics applied.  

```python
from maya import cmds

cmds.loadPlugin("boneDynamicsNode.mll", qt=True)

cmds.select(cl=True)
bon = cmds.joint(p=[0,0,0])
end = cmds.joint(p=[10,0,0])

bd_node = cmds.createNode("boneDynamicsNode")
cmds.connectAttr('time1.outTime', f'{bd_node}.time', f=True)
cmds.connectAttr(f'{bon}.translate', f'{bd_node}.boneTranslate', f=True)
cmds.connectAttr(f'{bon}.parentMatrix[0]', f'{bd_node}.boneParentMatrix', f=True)
cmds.connectAttr(f'{bon}.parentInverseMatrix[0]', f'{bd_node}.boneParentInverseMatrix', f=True)
cmds.connectAttr(f'{bon}.jointOrient', f'{bd_node}.boneJointOrient', f=True)
cmds.connectAttr(f'{end}.translate', f'{bd_node}.endTranslate', f=True)
cmds.connectAttr(f'{bd_node}.outputRotate', f'{bon}.rotate', f=True)

cmds.currentTime(1)
cmds.select(bon)
```

> Please set "Playback speed" to **Play every frame**.

Refer to [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start) and [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage) for details.

## 📖 Documentation

Full documentation is available in the [wiki](https://github.com/akasaki1211/boneDynamicsNode/wiki/Home):

- [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation)
- [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start)
- [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage)
- [Dynamics Parameters](https://github.com/akasaki1211/boneDynamicsNode/wiki/Dynamics-Parameters)
- [Collision Guide](https://github.com/akasaki1211/boneDynamicsNode/wiki/Collision-Guide)
- [Advanced Features](https://github.com/akasaki1211/boneDynamicsNode/wiki/Advanced-Features)
- [Practical Examples](https://github.com/akasaki1211/boneDynamicsNode/wiki/Practical-Examples)
- [FAQ & Troubleshooting](https://github.com/akasaki1211/boneDynamicsNode/wiki/FAQ-&-Troubleshooting)
- [Change Log](https://github.com/akasaki1211/boneDynamicsNode/wiki/Change-Log)

## 📦 Pre-built plug-ins

Pre-built `boneDynamicsNode.mll` in the [plug-ins](./plug-ins) directory. Install to the appropriate Maya version and ready to use.  

|Version|Plug-in<br>(*Click on "Download raw file" at the link.)|
|---|---|
|Maya 2022 Update 5 win64|[Download](./plug-ins/2022/boneDynamicsNode.mll)|
|Maya 2023 Update 3 win64|[Download](./plug-ins/2023/boneDynamicsNode.mll)|
|Maya 2024 Update 2 win64|[Download](./plug-ins/2024/boneDynamicsNode.mll)|
|Maya 2025 Update 3 win64|[Download](./plug-ins/2025/boneDynamicsNode.mll)|
|Maya 2026 Update 1 win64|[Download](./plug-ins/2026/boneDynamicsNode.mll)|
