# The Skeleton editor

Double-click a skeleton in the asset browser (or `OpenResourceEditor -Asset <guid>`): the bones drawn in 3D, the hierarchy of every imported
bone with what the compiler will do with it, and the descriptor. What the compiler does with a bone (a new name, virtual, deleted, exposed as a
socket, its LOD, its weight in each mask layer) is edited in the hierarchy or with commands, all undoable.

```
source/Editor/xskeleton_editor.h          the editor (session), the bone edit commands, the scene with the bone fill and its shadow
source/Editor/xskeleton_editor_scene.h    the scene the skeleton, animation and skin editors share: camera, ground grid, bone lines
source/Editor/xskeleton_editor_view.h     how bones are drawn (wedges, root sphere, colours), and the ray tests picking uses
source/Editor/shaders/                    the bone fill's shaders and the light's depth-only pair
```

A host includes `xskeleton_editor.h` (it registers the editor for the `xSkeleton` type; the resource loader is compiled into the host by
`xskeleton_editor_scene.h`). The host provides an `xgpu::device` and the main `xgpu::window`: the light's view of the bone fills is drawn into a
shadow map with a render pass on the window before the frame's UI is rendered.

## Panels

| Panel | |
|---|---|
| Skeleton Viewport | the bones as octahedral wedges (outline, translucent fill, a sphere for a root); click selects (Ctrl toggles, Shift ranges), right drag turns, middle drag pans, the wheel zooms |
| Skeleton View | pose (frozen: the rest pose, bind: the pose the mesh was skinned in), names, resize to 1 m, colouring (normal, LOD, active layer, exposed), the active mask layer |
| Bone Hierarchy | every imported bone: Sock(et), Virt(ual), Del(ete), LOD and the weight in the active layer; double-click renames, right click for the menu. An edit applies to the whole selection when the bone is part of it |
| Description | the descriptor |

## Commands

Run as `<resource name>\<Command>`. Bones are named by their raw import name (the compiler matches overrides by it); several are given one per
line. Names, paths and values are base64.

| Command | |
|---|---|
| `ListBones` | every compiled bone: index, name, raw name, parent, flags, LOD, whether it is selected or marked for deletion |
| `SetBone -Bones [-Rename] [-Virtual b] [-Delete b] [-Expose b] [-LOD n]` | what the compiler does with the bones (undoable); a LOD is also pushed down to descendants that have a lower one |
| `MatchLOD -Bones` | every descendant gets the bone's own LOD (undoable) |
| `SetMaskWeight -Layer -Bones -Weight [-Children true]` | weight (0 to 1) of bones in a mask layer; 0 removes the entry (undoable) |
| `ListLayers` | the mask layers |
| `SelectBones -Bones [-Add true]`, `ClearSelection` | the selection (view state) |
| `ListProperties`, `SetProperty`, `ListPreview`, `SetPreview` | descriptor properties (undoable) and the view settings |
| `Save`, `Compile`, `Undo`, `Redo` | |
| `CompileStatus [-Lines n]` | how the last compile went: state, unsaved changes, validation errors, the end of the log |
| `SetCamera [-Yaw -Pitch -Distance -Target x,y,z]`, `GetCamera`, `FrameSubject` | the preview camera (degrees), read back, or refitted to the subject (view state, not undoable) |

Picking casts a ray from the mouse against the bones' solid shapes on the CPU. (The E23 example also picks by drawing bone ids on the GPU; that
is not part of the editor.)
