#!/usr/bin/env python3
"""Render source-of-truth STL assembly in mm (not a hand-drawn concept image)."""
from pathlib import Path
import json
import vtk

ROOT = Path(__file__).resolve().parents[1]
CAD = Path(__file__).parent
PARTS = json.loads((CAD / "assembly.json").read_text())["parts"]


def stl_actor(path, position, color, orientation=None, opacity=1.0):
    source = vtk.vtkSTLReader()
    source.SetFileName(str(path))
    source.Update()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetPosition(*position)
    if orientation is not None:
        actor.SetOrientation(*orientation)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetOpacity(opacity)
    return actor


def box_actor(dimensions, at, color, opacity=1.0):
    source = vtk.vtkCubeSource()
    source.SetXLength(dimensions[0])
    source.SetYLength(dimensions[1])
    source.SetZLength(dimensions[2])
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetPosition(*at)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    return actor


def new_scene():
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.965, 0.973, 0.988)
    for name, part in PARTS.items():
        path = ROOT / part["file"]
        if name.startswith(("20_wheel", "21_tire")):
            for x in (-122, 122):
                for y in (-69, 69):
                    h = 15 if name.startswith("20_") else 17
                    color = (0.62, 0.65, 0.70) if h == 15 else (0.085, 0.10, 0.12)
                    renderer.AddActor(stl_actor(path, (x, y + h, 35), color, (90, 0, 0)))
        elif name.startswith("19_"):
            continue  # Adjustable spare drag-link is not installed in the default assembly.
        else:
            color = (
                (0.12, 0.28, 0.75) if "chassis" in name else
                (0.46, 0.48, 0.52) if "splice" in name else
                (0.80, 0.82, 0.85) if "deck" in name else
                (0.20, 0.22, 0.25) if "servo" in name else
                (0.54, 0.57, 0.61) if "knuckle" in name or "bearing" in name else
                (0.34, 0.40, 0.48)
            )
            renderer.AddActor(stl_actor(path, part["origin_mm"], color))
    # Physical envelopes: not fabricated from STL. Each corresponds to the
    # same electronics block in the SDF / URDF so physical fit can be checked.
    hardware = [
        ((40.7,19.7,42.9),(82,0,85),(0.13,0.15,0.18)),
        ((61,31,31),(-90,-26,80),(0.74,0.75,0.78)),
        ((49,39,36),(-112,26,82),(0.92,0.62,0.20)),
        ((80,44,33),(-5,0,91),(0.22,0.22,0.25)),
        ((85,56,16),(9,27,135),(0.13,0.61,0.28)),
        ((50,28,17),(-30,-45,135),(0.83,0.19,0.20)),
        ((23,27,23),(146,0,153),(0.18,0.18,0.2)),
        ((42,42,34),(80,0,183),(0.94,0.58,0.15)),
    ]
    for dims, pos, color in hardware:
        renderer.AddActor(box_actor(dims, pos, color, .90))
    return renderer


def render(filename, camera, look_at, parallel_scale):
    renderer = new_scene()
    cam = renderer.GetActiveCamera()
    cam.ParallelProjectionOn()
    cam.SetPosition(*camera)
    cam.SetFocalPoint(*look_at)
    cam.SetViewUp(0,1,0) if filename.endswith("_top.png") else cam.SetViewUp(0,0,1)
    cam.SetParallelScale(parallel_scale)
    renderer.SetActiveCamera(cam)
    win = vtk.vtkRenderWindow()
    win.OffScreenRenderingOn()
    win.SetSize(1500, 1100)
    win.SetMultiSamples(0)
    win.AddRenderer(renderer)
    win.Render()
    image = vtk.vtkWindowToImageFilter()
    image.SetInput(win)
    image.Update()
    png = vtk.vtkPNGWriter()
    png.SetFileName(str(CAD / filename))
    png.SetInputConnection(image.GetOutputPort())
    png.Write()
    win.Finalize()
    print(filename, (CAD / filename).stat().st_size, "bytes")


if __name__ == "__main__":
    render("rcauto_350x200_isometric.png", (490,-520,405), (0,0,100), 220)
    render("rcauto_350x200_top.png", (0,0,700), (0,0,100), 185)
    render("rcauto_350x200_side.png", (50,-650,185), (0,0,100), 160)
