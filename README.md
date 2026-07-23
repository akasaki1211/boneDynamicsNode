# boneDynamicsNode

![Maya](https://img.shields.io/static/v1?message=Maya&color=0696D7&logo=Autodesk&logoColor=white&label=) ![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)

**boneDynamicsNode** is a custom node for Autodesk Maya that enables dynamic simulation on joint chains. It is designed for easy integration into rigs and supports features such as collision handling, angle limits, target pose manipulation, and more.

> [!WARNING]
> This plug-in uses internal-use Maya node IDs:  
> `0x7b001`, `0x7b002`, `0x7b003`, `0x7b004`, and `0x7b009`.  

## ✨ Features

- Simple integration with standard Maya joints
- Multiple collision types: sphere, capsule, infinite plane, ground, and mesh
- Dedicated collider nodes and visualizer node
- Python helper package for scripted setup
- Angle limits
- Target pose control
- Transform cancellation for specific nodes
- Per-section scaling
- Turbulent wind
- Support for branching structures

## 🔧 Installation

Download the release zip from the [Releases page](https://github.com/akasaki1211/boneDynamicsNode/releases).

After extracting the zip file:

1. Copy the `boneDynamicsNode.mll` file for your Maya version from the `plug-ins` folder to `C:\Users\<USERNAME>\Documents\maya\<MAYAVERSION>\plug-ins`.
2. Copy the `scripts` folder to `C:\Users\<USERNAME>\Documents\maya`.  
   - The `scripts` folder installs the `bdn` Python helper package and `AEboneDynamicsNodeTemplate.mel`.

For additional instructions, refer to the [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation) wiki page.

## 🖥️ Platform Status

- **Windows:** Developed and tested on Windows. Pre-built plug-ins are provided for Windows.
- **Linux:** Build verified under WSL. Runtime testing in Maya on Linux has not been performed.
- **macOS:** Not tested.

## 🚀 Try it out

Please try running the following script. It creates a single-section joint with bone dynamics applied.  

```python
from maya import cmds

cmds.select(cl=True)
bon = cmds.joint(p=[0,0,0])
end = cmds.joint(p=[10,0,0])

import bdn
bdn.create_dynamics_node(bon, end)

cmds.currentTime(1)
cmds.select(bon)
```

> [!IMPORTANT]
> Please set "Playback speed" to **Play every frame**.

Refer to [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start) and [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage) for details.

## 📖 Documentation

Full documentation is available in the [wiki](https://github.com/akasaki1211/boneDynamicsNode/wiki/Home):

- [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation)
- [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start)
- [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage)
- [Dynamics Parameters](https://github.com/akasaki1211/boneDynamicsNode/wiki/Dynamics-Parameters)
- [Visualizer](https://github.com/akasaki1211/boneDynamicsNode/wiki/Visualizer)
- [Collision Guide](https://github.com/akasaki1211/boneDynamicsNode/wiki/Collision-Guide)
- [Advanced Features](https://github.com/akasaki1211/boneDynamicsNode/wiki/Advanced-Features)
- [Practical Examples](https://github.com/akasaki1211/boneDynamicsNode/wiki/Practical-Examples)
- [FAQ & Troubleshooting](https://github.com/akasaki1211/boneDynamicsNode/wiki/FAQ-&-Troubleshooting)
- [Change Log](https://github.com/akasaki1211/boneDynamicsNode/wiki/Change-Log)

## 📦 Individual Plug-ins

If you only need to open and evaluate scenes that are already set up with boneDynamicsNode, the main `.mll` plug-in file is enough.

Pre-built `boneDynamicsNode.mll` files are available in the [plug-ins](./plug-ins) directory. Download the file that matches your Maya version and place it in your Maya plug-ins path.

**Click "Download raw file" from the linked file page.**

- [Maya 2022 Update 5 Win64](./plug-ins/2022/boneDynamicsNode.mll)
- [Maya 2023 Update 3 Win64](./plug-ins/2023/boneDynamicsNode.mll)
- [Maya 2024 Update 2 Win64](./plug-ins/2024/boneDynamicsNode.mll)
- [Maya 2025 Update 3 Win64](./plug-ins/2025/boneDynamicsNode.mll)
- [Maya 2026 Update 3 Win64](./plug-ins/2026/boneDynamicsNode.mll)
- [Maya 2027 Update 1 Win64](./plug-ins/2027/boneDynamicsNode.mll)

If you want to create new setups using the `bdn` Python helper package or use the Attribute Editor template, follow the instructions in the [Installation](#-installation) section.
