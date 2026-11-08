#!/usr/bin/env python3
"""Parametric, native-millimetre STL generation for RCauto Ackermann 350 x 200 mm.

Coordinate system: +X forward, +Y left, +Z up. World pose positions correspond
to ROS2/Gazebo SDF; exported STL origins are defined in assembly.json.
Run from RCauto root using cad/build.sh; installed Gazebo meshes use 0.001 scale.
Unspecified motor/ESC/gearbox mount patterns are adjustable, not device-specific.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "sim" / "rcauto_350x200" / "meshes"
CAD = Path(__file__).resolve().parent
OUT.mkdir(parents=True, exist_ok=True)
PARTS = {}
RES = 48
M3 = 3.4
M25 = 2.9
M4 = 4.4
M5 = 5.3


def box(x, y, z, at=(0, 0, 0)):
    mesh = trimesh.creation.box(extents=[x, y, z])
    mesh.apply_translation(at)
    return mesh


def cyl(radius, height, at=(0, 0, 0), axis="z"):
    mesh = trimesh.creation.cylinder(radius=radius, height=height, sections=RES)
    if axis == "y":
        mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    elif axis == "x":
        mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    mesh.apply_translation(at)
    return mesh


def join(*solids):
    solids = [s for s in solids if s is not None]
    if len(solids) == 1:
        return solids[0]
    return trimesh.boolean.union(solids, engine="manifold", check_volume=True)


def drill(mesh, bores):
    return trimesh.boolean.difference([mesh, *bores], engine="manifold", check_volume=True)


def hole_z(x, y, r, height=300, z=90):
    return cyl(r, height, (x, y, z))


def part(name, solid, origin, material="PETG", quantity=1, role="", printing="flat", components=None):
    if not solid.is_watertight or not solid.is_volume or solid.volume <= 0:
        raise ValueError(f"Invalid printable mesh for {name}: watertight={solid.is_watertight}, vol={solid.volume}")
    local = solid.copy()
    local.apply_translation(-np.array(origin, dtype=float))
    local.remove_unreferenced_vertices()
    path = OUT / f"{name}.stl"
    local.export(path)
    if max(local.extents[:2]) > 220.1:
        raise ValueError(f"{name}: exceeds 220mm print bed: {local.extents}")
    PARTS[name] = {
        "file": str(path.relative_to(ROOT)),
        "origin_mm": [round(float(v), 4) for v in origin],
        "bounds_mm": [[round(float(v), 3) for v in row] for row in solid.bounds],
        "size_mm": [round(float(v), 3) for v in local.extents],
        "volume_cm3": round(float(solid.volume) / 1000, 3),
        "triangles": int(len(local.faces)),
        "watertight": bool(local.is_watertight),
        "material": material,
        "quantity": quantity,
        "printing": printing,
        "role": role,
        "visual_in_gazebo": bool(components),
        "body": components or "",
    }
    print(f"{name:<30} {len(local.faces):>6} faces | {local.volume/1000:7.2f} cm3 | {'x'.join(f'{v:.1f}' for v in local.extents)} mm")


def holes(mesh, locations, size=M3, z=50, length=30):
    return drill(mesh, [hole_z(x, y, size / 2, length, z) for x, y in locations])


def front_outline():
    return join(
        box(27, 145, 8, (68.5, 0, 56)),
        box(83, 92, 8, (123.5, 0, 56)),
        box(10, 145, 8, (170, 0, 56)),
        box(9, 94, 8, (161, 0, 56)),
    )


def rear_outline():
    return join(
        box(27, 145, 8, (-68.5, 0, 56)),
        box(83, 92, 8, (-123.5, 0, 56)),
        box(10, 145, 8, (-170, 0, 56)),
        box(9, 94, 8, (-161, 0, 56)),
    )


# Split chassis assembly; seam bars are fitted UNDER the deck at +/-55mm.
front = front_outline()
front = holes(front, [(67, y) for y in (-30, 30)] +
              [(65, y) for y in (-38, 38)] +
              [(x, y) for x in (57, 107) for y in (-14, 14)] +
              [(x, y) for x in (111, 133) for y in (-30, 30)] +
              [(x, y) for x in (134, 156) for y in (-11, 11)])
part("01_chassis_front", front, (115, 0, 52), role="Front 120mm module with wheel cutouts and integrated bumper", components="base_link")

center = box(110, 145, 8, (0, 0, 56))
center = holes(center, [(x, y) for x in (-43, 43) for y in (-30, 30)] +
               [(x, y) for x in (-35, 35) for y in (-50, 50)] +
               [(-35, -20), (25, 20)])
part("02_chassis_center", center, (0, 0, 52), role="Central 110mm chassis, battery mounting and cable slots", components="base_link")

rear = rear_outline()
rear = holes(rear, [(-67, y) for y in (-30, 30)] +
             [(-65, y) for y in (-59, 59)] +
             [(x, y) for x in (-119, -69) for y in (-37, -15)] +
             [(x, y) for x in (-134, -90) for y in (14, 36)] +
             [(x, y) for x in (-133, -111) for y in (-30, 30)])
part("03_chassis_rear", rear, (-115, 0, 52), role="Rear 120mm module with wheel cutouts and integrated bumper", components="base_link")

for side, cx in (("front", 55), ("rear", -55)):
    for position, cy in (("left", 30), ("right", -30)):
        strap = box(46, 13, 4, (cx, cy, 50))
        strap = holes(strap, [(cx - 12, cy), (cx + 12, cy)], z=50, length=10)
        part(f"04_splice_{side}_{position}", strap, (cx, cy, 48),
             role="M3 two-hole underside seam splice", components="base_link")

# Upper deck, real Raspberry Pi 5 58x49 hole pattern. Controller board is
# nominally 85x56mm; hole spacing from official mechanical drawing.
upper = box(205, 142, 5, (15, 0, 120))
upper = holes(upper, [(x, y) for x in (-30, 28) for y in (2.5, 51.5)],
              size=M25, z=120, length=10)
upper = holes(upper, [(65, y) for y in (-38, 38)] +
                     [(-65, y) for y in (-59, 59)], z=120, length=10)
# Slotted ESC strap/cable openings (50x28mm ESC is provisional).
upper = drill(upper, [box(9, 3, 9, (x, -45 + dy, 120))
                      for x in (-49, -11) for dy in (-11, 11)])
# Cable passage away from mounting positions.
upper = drill(upper, [box(18, 7, 9, (70, -12, 120))])
part("05_upper_deck_pi5_esc", upper, (15, 0, 117.5),
     role="205x142x5mm upper platform, Pi5 58x49 mounting, ESC straps, cable pass-through",
     components="upper_deck")

# Four long structural standoffs under the upper deck.
for side, x in (("front", 65), ("rear", -65)):
    for position, y in (("left", 38 if side == "front" else 59),
                        ("right", -38 if side == "front" else -59)):
        post = drill(cyl(6.0, 57.5, (x, y, 88.75)), [hole_z(x, y, M3 / 2, 66, 89)])
        part(f"06_standoff_{side}_{position}", post, (x, y, 60),
             role="57.5mm standoff with M3 through-hole; clamp using long screw and nut",
             components="base_link")

# Raspberry Pi 5 4.5mm standoff/spacer; holes pass M2.5 screws.
for n, (x, y) in enumerate([(x, y) for x in (-30, 28) for y in (2.5, 51.5)], 1):
    washer = drill(cyl(4, 4.5, (x, y, 124.75)), [hole_z(x, y, M25/2, 10, 124.75)])
    part(f"07_pi5_spacer_{n}", washer, (x, y, 122.5), "PETG",
         role="Pi5 M2.5 standoff; use nonconductive washers and check underside clearances", components="base_link")

# Servo MG996R adjustable cradle with top clamping bars.
servo = join(
    box(62, 40, 3, (82, 0, 61.5)),
    box(60, 5.0, 30, (82, -14.0, 76.7)),
    box(60, 5.0, 30, (82, 14.0, 76.7)),
)
# Clamping holes in side rails; clear MG996R body (40.7x19.7x42.9).
servo = holes(servo, [(x, y) for x in (57, 107) for y in (-14, 14)], size=M3, z=61.5, length=10)
servo = holes(servo, [(x, y) for x in (57.5, 106.5) for y in (-14, 14)],
              size=M3, z=83.5, length=29)
part("08_servo_MG996R_cradle", servo, (82, 0, 60),
     role="MG996R 40.7x19.7 footprint, clamp slots; do not print spline", components="base_link")
for label, x in (("front", 106.5), ("rear", 57.5)):
    clamp = box(7, 40, 3.5, (x, 0, 94.5))
    clamp = holes(clamp, [(x, -14), (x, 14)], size=M3, z=94, length=10)
    part(f"09_servo_clamp_{label}", clamp, (x, 0, 92.75),
         role="Servo top clamp; M3 machine screws", components="base_link")

# Universal BLDC round-motor saddle, adjustable straps. The motor dimensions
# are placeholders until the exact motor shaft/mount diameter is known.
motor = join(
    box(68, 36, 4, (-90, -26, 62)),
    box(56, 4, 19, (-90, -41.2, 73.0)),
    box(56, 4, 19, (-90, -10.8, 73.0)),
)
motor = drill(motor, [cyl(15.8, 80, (-90, -26, 80), "x")])
motor = holes(motor, [(-119, -26), (-61, -26)], size=M3, z=64, length=14)
part("10_bldc_adjustable_cradle", motor, (-90, -26, 60),
     role="Approx 31mm Ø motor saddle with strap mounts; confirm real motor size", components="base_link")

gear = join(
    box(60, 40, 4, (-112, 26, 62)),
    box(4, 40, 14, (-140, 26, 71)),
    box(4, 40, 14, (-84, 26, 71)),
)
gear = holes(gear, [(-134, 14), (-90, 36)], z=62, length=10)
part("11_gearbox_universal_tray", gear, (-112, 26, 60),
     role="Generic 49x39mm gearcase envelope; adjustable, no claimed fit for unspecified gearbox",
     components="base_link")

battery = join(
    box(90, 50, 4, (-5, 0, 72.5)),
    *[box(7, 7, 12.5, (-5 + xx, yy, 66.25))
      for xx in (-35, 35) for yy in (-18, 18)],
    box(90, 2.5, 9, (-5, -24, 79)),
    box(90, 2.5, 9, (-5, 24, 79)),
)
battery = holes(battery, [(-35, -20), (25, 20)], size=M3, z=72, length=35)
part("12_battery_lifted_tray", battery, (-5, 0, 60),
     role="Battery tray for approx 80x44mm pack; strap and regulator wiring needed",
     components="base_link")

esc = join(box(56, 34, 4, (-30, -45, 124.5)),
           box(56, 2.2, 8, (-30, -60.8, 128)),
           box(56, 2.2, 8, (-30, -29.2, 128)))
esc = drill(esc, [box(9, 3.5, 8, (x, -45 + dy, 126))
                  for x in (-49, -11) for dy in (-11, 11)])
part("13_esc_strap_tray", esc, (-30, -45, 122.5),
     role="Universal 50x28mm ESC strap cradle; verify ESC heatsink/airflow", components="base_link")

# Camera mast at the front is outside the upper deck; cross-section braces.
camera = join(
    box(34, 31, 4, (145, 0, 62)),
    box(10, 16, 77, (135, 0, 102.5)),
    box(30, 29, 4, (145, 0, 139.5)),
    box(11, 12, 15, (134, 0, 70)),
)
camera = holes(camera, [(x, y) for x in (134, 156) for y in (-11, 11)], z=63, length=11)
part("14_camera_front_mast", camera, (145, 0, 60),
     role="83mm front mast supporting assumed 23x27mm camera module", components="base_link")

lidar = join(box(42, 42, 4, (80, 0, 124.5)),
             cyl(7, 39, (80, 0, 145.5)),
             box(43, 43, 4, (80, 0, 164)))
lidar = holes(lidar, [(x, y) for x in (67, 93) for y in (-13, 13)],
              z=125, length=9)
part("15_lidar_pedestal", lidar, (80, 0, 122.5),
     role="Lidar raised to body z166mm; mating holes vary by actual sensor", components="base_link")


def fixed_kingpin_bracket(sign):
    x, y = 122, sign * 44
    inner = sign * 30
    upper = cyl(7.3, 6, (x, y, 48))
    lower = cyl(7.3, 6, (x, y, 23))
    web = box(11, 10, 30, (x - 9, sign * 26, 36))
    lower_bridge = box(11, 20, 6, (x - 5.5, sign * 35, 23))
    flange = box(32, 22, 4, (x, inner, 50))
    boss = join(upper, lower, web, flange, lower_bridge)
    bores = [hole_z(x, y, M4 / 2, 45, 35),
             hole_z(x-11, inner, M3 / 2, 12, 51),
             hole_z(x+11, inner, M3 / 2, 12, 51)]
    return drill(boss, bores)


for sign, suffix in ((1, "left"), (-1, "right")):
    br = fixed_kingpin_bracket(sign)
    part(f"16_kingpin_fork_{suffix}", br, (122, sign*44, 20),
         role="Front-steering bearing fork; M4 metal kingpin and M3 chassis fastening",
         printing="sideways with supports", components="base_link")


def knuckle(sign):
    # Vertical kingpin at (122,+/-44,35); axle centre offsets +/-25mm to tyre centre.
    yp = sign * 44
    pivot = cyl(6.5, 17, (122, yp, 35.5))
    axle = cyl(9, 8, (122, sign * 49, 35), "y")
    bridge = box(11, 9, 9, (122, sign * 48.5, 35))
    # Angled steering arm geometry points toward centre of rear axle.
    arm = box(21, 6.5, 7, (112, sign * 41.5, 35))
    lug = cyl(5.2, 7, (104, sign * 39, 35))
    solid = join(pivot, axle, bridge, arm, lug)
    return drill(solid, [
        hole_z(122, yp, M4 / 2, 30, 35),
        cyl(M5/2, 36, (122, sign * 55, 35), "y"),
        hole_z(104, sign * 39, M3 / 2, 15, 35),
    ])


for sign, suffix in ((1, "left"), (-1, "right")):
    kn = knuckle(sign)
    part(f"17_front_knuckle_{suffix}", kn, (122, sign*44, 27),
         role="Ackermann steering knuckle with M4 kingpin, M5 axle, M3 tie-rod eye",
         printing="sideways, support hub", components=f"front_{suffix}_knuckle")

# Printed single-piece tie rod with adjustable metal ball joints recommended.
tie = join(box(7.5, 78, 4.5, (104, 0, 41)),
           cyl(5.2, 4.5, (104, 39, 41)),
           cyl(5.2, 4.5, (104, -39, 41)),
           box(10, 11, 4.5, (104, 0, 41)))
tie = holes(tie, [(104, -39), (104, 0), (104, 39)], size=M3, z=41, length=12)
part("18_steering_tie_rod", tie, (104, 0, 38.75),
     role="Linkage eye pitch 78mm, M3 swivel/rod-end bolts; test for binding",
     components="base_link")
# Servo drag link is trim-to-fit depending supplied horn/spline shape.
drag = join(box(7, 38, 4, (0, 0, 2)), cyl(5.5, 4, (0, -19, 2)), cyl(5.5, 4, (0, 19, 2)))
drag = holes(drag, [(0, -19), (0, 19)], z=2, length=10)
part("19_servo_drag_link_38mm", drag, (0, 0, 0),
     role="Reference link; use stock servo horn and spherical joints", components=None)


def wheel_rim():
    outer = drill(cyl(31, 30, (0, 0, 15)), [cyl(24.8, 34, (0, 0, 15))])
    hub = cyl(11.8, 30, (0, 0, 15))
    spokes = []
    for i in range(6):
        spoke = box(18, 6.4, 21, (16, 0, 15))
        spoke.apply_transform(trimesh.transformations.rotation_matrix(math.radians(i * 60), [0, 0, 1]))
        spokes.append(spoke)
    lips = [drill(cyl(32, 1.5, (0, 0, z)), [cyl(24.8, 4, (0, 0, z))])
            for z in (0.75, 29.25)]
    solid = join(outer, hub, *spokes, *lips)
    # 625ZZ bearing: 5x16x5; 0.2mm printed pocket diametral clearance.
    solid = drill(solid, [
        cyl(M5/2, 35, (0, 0, 15)),
        cyl(8.1, 5.2, (0, 0, 2.6)),
        cyl(8.1, 5.2, (0, 0, 27.4)),
    ])
    return solid


rim = wheel_rim()
part("20_wheel_rim_625ZZ", rim, (0, 0, 0), "PETG", 4,
     role="Ø62mm rim, 30mm wide; two 625ZZ 5x16x5 bearings per wheel", components="wheel_rim")

tyre = drill(cyl(35, 34, (0, 0, 17)), [cyl(30.8, 40, (0, 0, 17))])
part("21_tire_70x34_TPU", tyre, (0, 0, 0), "TPU95A", 4,
     role="Ø70mm tire, 34mm wide; elastic stretch over rim lips", components="wheel_tire")

# Rear bearing supports for 5mm metal half-shafts driven by a REAL differential.
for sign, suffix in ((1, "left"), (-1, "right")):
    y=sign*43
    support = join(
        box(29, 22, 4, (-122, sign*31, 50)),
        box(11, 11, 26, (-122, sign*39, 37)),
        cyl(11.5, 10, (-122, sign*46, 35), "y"),
    )
    support=drill(support, [
        cyl(8.1, 12, (-122, sign*46, 35), "y"),
        hole_z(-133, sign*31, M3/2, 12, 50),
        hole_z(-111, sign*31, M3/2, 12, 50),
    ])
    part(f"22_rear_bearing_hanger_{suffix}", support, (-122, sign*43, 23),
         role="Rear 625ZZ bearing carrier at z35, steel shaft required",
         printing="sideways, supports", components="base_link")

# Readable, machine-actionable part count, poses and print details.
manifest={
    "project":"RCauto 350x200 Ackermann",
    "units":"mm",
    "frame":"world; +X forward,+Y left,+Z up",
    "design_envelope_mm":[350,200],
    "wheelbase_mm":244,
    "rear_tyre_track_mm":138,
    "steering_kingpin_track_mm":88,
    "wheel_outer_diameter_mm":70,
    "wheel_width_mm":34,
    "assembled_plate_height_mm":52,
    "nominal_components":{
        "towerpro_servo":"MG996R 40.7 x 19.7 x 42.9 mm; factory horn and fasteners required",
        "raspberry_pi5":"85 x 56 mm; 58 x 49 mm M2.5 mounting pattern (from official drawing)",
        "bldc_motor":"Envelope 61 x 31 x 31 mm is ASSUMED",
        "gearbox":"Envelope 49 x 39 x 36 mm is ASSUMED",
        "esc":"Envelope 50 x 28 x 17 mm is ASSUMED",
        "battery":"Envelope 80 x 44 x 33 mm is ASSUMED",
    },
    "required_purchased_hardware":[
        "4 x Ø4mm steel kingpin with washers/locknuts; lengths verify after dry fit",
        "M3 bolts and nyloc nuts for modular chassis seams and steering ball-joints",
        "M2.5 bolts for Raspberry Pi 5 standoffs and nonconductive insulation",
        "8 x 625ZZ (5x16x5 mm) ball bearings, or replace for actual axle choice",
        "5mm steel wheel axles, real rear differential and drivetrain couplers",
        "Factory TowerPro servo horn; powered regulated BEC; physical E-stop",
        "BLDC motor, compatible gearbox/differential, ESC and battery harness",
    ],
    "printing":{
        "stl_units":"millimetres",
        "suggested_bed_mm":[220,220],
        "main_structure":"PETG or PA-CF with appropriate material-specific settings; 4+ walls, 35-50% infill",
        "wheels":"TPU 95A tyres, PETG/PA rims; 4+ walls",
        "seam_fasteners":"M3 machine bolts + washers + locknuts",
        "tolerance":"0.3mm nominal FDM fit clearance; test print critical holes, ream bearings as required",
    },
    "status":"PRINTABLE PROTOTYPE; final drivetrain shaft, diff, horn/gearbox and electronics fit require physical measurement and test",
    "parts":PARTS,
}
(CAD/"assembly.json").write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+"\n")
print(f"TOTAL: {len(PARTS)} distinct STL filenames; printable items = {sum(p['quantity'] for p in PARTS.values())}")
