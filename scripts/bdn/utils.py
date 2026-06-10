from typing import Dict, Tuple, Sequence, Any
import traceback
import functools

from maya import cmds

# decorator
def undo_chunk(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        cmds.undoInfo(openChunk=True)
        try:
            result = func(*args, **kwargs)
        finally:
            cmds.undoInfo(closeChunk=True)
        return result
    return wrapper


def with_traceback(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            print(f"Error occurred in '{func.__name__}'")
            traceback.print_exc()
            raise
    return wrapper


# utils
def load_plugin(*args) -> bool:
    try:
        result = cmds.loadPlugin("boneDynamicsNode.mll", qt=True)
        if result is not None:
            plugin_ver = cmds.pluginInfo("boneDynamicsNode.mll", q=True, version=True)
            print(f"Plugin loaded successfully. Version: {plugin_ver}")
        return True
    except Exception as e:
        print(f"Error loading plugin: {e}")
        return False


def get_fps(*args) -> float:
    time_unit = cmds.currentUnit(q=True, time=True)
    fps_mapping = {
        'game': 15.0,
        'film': 24.0,
        'pal': 25.0,
        'ntsc': 30.0,
        'show': 48.0,
        'palf': 50.0,
        'ntscf': 60.0
    }
    return fps_mapping.get(time_unit, 24.0)


@with_traceback
@undo_chunk
def create_shape_node(node_name: str, *args) -> Tuple[str, str]:
    shape = cmds.createNode(node_name, name=f"{node_name}Shape")
    transform = cmds.listRelatives(shape, parent=True, fullPath=False)[0]
    if not transform.startswith(node_name):
        transform = cmds.rename(transform, node_name)
    shape = cmds.rename(shape, transform + "Shape")
    return transform, shape


@with_traceback
@undo_chunk
def set_attributes(node: str, **attributes: Dict[str, Any]) -> None:
    for at, v in attributes.items():
        if not cmds.attributeQuery(at, n=node, ex=True):
            continue
        try:
            if type(v) == str:
                cmds.setAttr(f"{node}.{at}", v, type="string")
            
            elif (type(v) == list or type(v) == tuple) and len(v) >= 3:
                cmds.setAttr(f"{node}.{at}", v[0], v[1], v[2], type="double3")
            
            else:
                cmds.setAttr(f"{node}.{at}", v)
        except:
            print(f"Failed to set attribute: {node}.{at}")


@with_traceback
@undo_chunk
def set_outliner_color(node: str, color: Sequence[float], *args) -> None:
    cmds.setAttr(f'{node}.useOutlinerColor', True)
    cmds.setAttr(f'{node}.outlinerColor', color[0], color[1], color[2])


@with_traceback
@undo_chunk
def set_override_display_color(node: str, color_idx: int, *args) -> None:
    cmds.setAttr(f'{node}.overrideEnabled', True)
    cmds.setAttr(f'{node}.overrideRGBColors', False)
    cmds.setAttr(f'{node}.overrideColor', color_idx)


@with_traceback
@undo_chunk
def set_override_display_type(node: str, type: str, *args) -> None:
    if type == 'Reference':
        display_type = 2
    elif type == 'Template':
        display_type = 1
    else :
        display_type = 0

    set_override_display_color(node, 0)
    cmds.setAttr(f'{node}.overrideDisplayType', display_type)


@with_traceback
@undo_chunk
def add_object_to_set(obj: str, set_name: str, *args) -> None:
    
    if not set_name:
        return
    
    if not cmds.objExists(set_name):
        cmds.select(cl=True)
        cmds.sets(name=set_name)
    
    cmds.sets(obj, add=set_name)