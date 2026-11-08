#!/usr/bin/env python3
"""Sync printable CAD meshes into ROS Xacro and Gazebo SDF; safe to rerun."""
from pathlib import Path
import json
from lxml import etree as E

ROOT = Path(__file__).resolve().parents[1]
SDF_PATH = ROOT / "sim/rcauto_350x200/model.sdf"
URDF_PATH = ROOT / "description/rcauto_350x200.urdf.xacro"
PARTS = json.loads((Path(__file__).parent / "assembly.json").read_text())["parts"]
NS = "{http://www.ros.org/wiki/xacro}"
SCALE = "0.001 0.001 0.001"
PACK = "package://rcauto_cpp/sim/rcauto_350x200/meshes/"
MODEL = "model://rcauto_350x200/meshes/"
D = chr(36)


def val(v):
    return ("%.8f" % v).rstrip("0").rstrip(".") if abs(v)>1e-9 else "0"


def offset(spec, point):
    return " ".join(val((a-b)/1000) for a,b in zip(spec["origin_mm"],point))


def mesh(parent, name, xyz, link_origin, sdf):
    vis=E.SubElement(parent,"visual",name="print_"+name)
    p=offset({"origin_mm":xyz},link_origin)
    if sdf:
        E.SubElement(vis,"pose").text=p+" 0 0 0"
        geo=E.SubElement(vis,"geometry")
        m=E.SubElement(geo,"mesh")
        E.SubElement(m,"uri").text=MODEL+name+".stl"
        E.SubElement(m,"scale").text=SCALE
        mat=E.SubElement(vis,"material")
        color="0.13 0.32 0.80 1" if "chassis" in name else "0.62 0.64 0.68 1"
        E.SubElement(mat,"ambient").text=color
        E.SubElement(mat,"diffuse").text=color
    else:
        E.SubElement(vis,"origin",xyz=p)
        E.SubElement(E.SubElement(vis,"geometry"),"mesh",filename=PACK+name+".stl",scale=SCALE)
        E.SubElement(vis,"material",name="blue" if "chassis" in name else "metal")


def update_sdf():
    tree=E.parse(str(SDF_PATH))
    model=tree.getroot().find("model")
    base=model.find("./link[@name='base_link']")
    if base is None: raise ValueError("base_link missing")
    for link in model.findall("link"):
        for visual in list(link.findall("visual")):
            if visual.get("name","").startswith("print_"):
                link.remove(visual)
    obsolete={"lower_deck_visual","front_bumper_visual","rear_bumper_visual",
              "upper_deck_visual","post_fl_visual","post_fr_visual","post_rl_visual",
              "post_rr_visual","lidar_mast_visual","camera_bracket_visual",
              "rear_axle_housing_visual"}
    for v in list(base.findall("visual")):
        if v.get("name") in obsolete: base.remove(v)
    replacements={
        "towerpro_mg996r_visual":"0.082 0 0.029 0 0 0",
        "gearbox_diff_visual":"-0.112 0.026 0.026 0 0 0",
        "battery_visual":"-0.005 0 0.035 0 0 0",
    }
    for name,pose in replacements.items():
        v=base.find("./visual[@name='"+name+"']")
        if v is None: raise ValueError(name)
        v.find("pose").text=pose
    # BLDC motor is round: the printed adjustable cradle has a 31.6mm bore.
    motor_visual=base.find("./visual[@name='bldc_motor_visual']")
    motor_visual.find("pose").text="-0.090 -0.026 0.024 0 1.57079632679 0"
    motor_geometry=motor_visual.find("geometry")
    for g in list(motor_geometry):motor_geometry.remove(g)
    barrel=E.SubElement(motor_geometry,"cylinder")
    E.SubElement(barrel,"radius").text="0.0155"
    E.SubElement(barrel,"length").text="0.061"
    lidar_visual=base.find("./visual[@name='front_lidar_visual']")
    lidar_visual.find("pose").text="0.080 0 0.127 0 0 0"
    lidar_sensor=base.find("./sensor[@name='lidar_front']")
    lidar_sensor.find("pose").text="0.080 0 0.148 0 0 0"
    for c in list(base.findall("collision")):base.remove(c)
    collision_boxes=[
        ("center",0,0,0,0.164,0.145,0.008),
        ("front",0.1235,0,0,0.083,0.092,0.008),
        ("rear",-0.1235,0,0,0.083,0.092,0.008),
        ("front_bumper",0.17,0,0,0.010,0.145,0.008),
        ("rear_bumper",-0.17,0,0,0.010,0.145,0.008),
    ]
    for name,x,y,z,sx,sy,sz in collision_boxes:
        c=E.SubElement(base,"collision",name=name+"_collision")
        E.SubElement(c,"pose").text=f"{x} {y} {z} 0 0 0"
        E.SubElement(E.SubElement(c,"geometry"),"box").append(E.Element("size"))
        c.find("./geometry/box/size").text=f"{sx} {sy} {sz}"
    for name,spec in PARTS.items():
        if spec["body"] in ("base_link","upper_deck"):
            mesh(base,name,spec["origin_mm"],[0,0,56],True)
    for side,sgn in [("left",1),("right",-1)]:
        kn=model.find("./link[@name='front_"+side+"_knuckle']")
        kn.find("pose").text=f"0.122 {val(0.044*sgn)} 0.035 0 0 0"
        for v in list(kn.findall("visual")):kn.remove(v)
        name="17_front_knuckle_"+side
        mesh(kn,name,PARTS[name]["origin_mm"],[122,44*sgn,35],True)
    plugin=model.find("./plugin[@name='gz::sim::systems::AckermannSteering']")
    for tag,value in [("kingpin_width","0.088"),("wheel_separation","0.138"),("steering_limit","0.35")]:
        plugin.find(tag).text=value
    for key in ("front_left","front_right","rear_left","rear_right"):
        link=model.find("./link[@name='"+key+"_wheel']")
        for v in list(link.findall("visual")):link.remove(v)
        for name,half,rgba in [
            ("20_wheel_rim_625ZZ",15,"0.58 0.58 0.60 1"),
            ("21_tire_70x34_TPU",17,"0.04 0.04 0.045 1"),
        ]:
            v=E.SubElement(link,"visual",name=key+"_"+name)
            E.SubElement(v,"pose").text=f"0 {val(half/1000)} 0 1.570796326795 0 0"
            m=E.SubElement(E.SubElement(v,"geometry"),"mesh")
            E.SubElement(m,"uri").text=MODEL+name+".stl"
            E.SubElement(m,"scale").text=SCALE
            mat=E.SubElement(v,"material")
            E.SubElement(mat,"ambient").text=rgba
            E.SubElement(mat,"diffuse").text=rgba
        for col in link.findall("collision"):
            shape=col.find("./geometry/cylinder")
            if shape is not None:
                shape.find("radius").text="0.035"
                shape.find("length").text="0.034"
    tree.write(str(SDF_PATH),encoding="UTF-8",xml_declaration=True,pretty_print=True)


def update_urdf():
    tree=E.parse(str(URDF_PATH))
    root=tree.getroot()
    base=root.find("./link[@name='base_link']")
    for el in list(base):
        if el.tag==NS+"box_visual" or (
            el.tag=="visual" and el.get("name","").startswith("print_")
        ):base.remove(el)
    for name,spec in PARTS.items():
        if spec["body"]=="base_link":
            mesh(base,name,spec["origin_mm"],[0,0,56],False)
    # Upper deck has its own fixed link in URDF, although Gazebo uses one lumped chassis.
    old=root.find("./"+NS+"mounted_box[@name='upper_deck']")
    if old is None:old=root.find("./"+NS+"mounted_mesh[@name='upper_deck']")
    if old is None:raise ValueError("upper_deck missing")
    old.tag=NS+"mounted_mesh"
    old.set("filename",PACK+"05_upper_deck_pi5_esc.stl")
    old.set("z_offset","-0.0025")
    if root.find("./"+NS+"macro[@name='mounted_mesh']") is None:
        m=E.Element(NS+"macro",name="mounted_mesh",
                    params="name px py pz x y z mass mat filename z_offset")
        k=D+"{"
        def v(name):return k+name+"}"
        l=E.SubElement(m,"link",name=v("name"))
        E.SubElement(l,NS+"box_inertia",mass=v("mass"),x=v("x"),y=v("y"),z=v("z"))
        visual=E.SubElement(l,"visual")
        E.SubElement(visual,"origin",xyz="0 0 "+v("z_offset"))
        E.SubElement(E.SubElement(visual,"geometry"),"mesh",
                     filename=v("filename"),scale=SCALE)
        E.SubElement(visual,"material",name=v("mat"))
        col=E.SubElement(l,"collision")
        E.SubElement(E.SubElement(col,"geometry"),"box",size=v("x")+" "+v("y")+" "+v("z"))
        j=E.SubElement(m,"joint",name=v("name")+"_mount",type="fixed")
        E.SubElement(j,"parent",link="base_link")
        E.SubElement(j,"child",link=v("name"))
        E.SubElement(j,"origin",xyz=v("px")+" "+v("py")+" "+v("pz"))
        root.insert(6,m)
    front_macro=root.find("./"+NS+"macro[@name='front_knuckle']")
    front_macro.set("params","side y wheel_offset")
    front_macro.find("./"+NS+"wheel").set("y",D+"{wheel_offset}")
    for instance in root.findall("./"+NS+"front_knuckle"):
        sign=1 if instance.get("side")=="left" else -1
        instance.set("y",D+"{kingpin_track/2}" if sign==1 else D+"{-kingpin_track/2}")
        instance.set("wheel_offset","0.025" if sign==1 else "-0.025")
    prop=root.find("./"+NS+"property[@name='kingpin_track']")
    if prop is None:root.insert(4,E.Element(NS+"property",name="kingpin_track",value="0.088"))
    else:prop.set("value","0.088")
    wmacro=root.find("./"+NS+"macro[@name='wheel']")
    wheel=wmacro.find("./link")
    for vis in list(wheel.findall("visual")):wheel.remove(vis)
    for name,half,material in [
        ("20_wheel_rim_625ZZ",15,"metal"),
        ("21_tire_70x34_TPU",17,"rubber"),
    ]:
        vis=E.SubElement(wheel,"visual")
        E.SubElement(vis,"origin",xyz=f"0 {val(half/1000)} 0",rpy=D+"{pi/2} 0 0")
        E.SubElement(E.SubElement(vis,"geometry"),"mesh",filename=PACK+name+".stl",scale=SCALE)
        E.SubElement(vis,"material",name=material)
    kn=front_macro.find("./link[@name='front_"+D+"{side}_knuckle']")
    if kn is None:raise ValueError("front knuckle macro")
    for vis in list(kn.findall("visual")):kn.remove(vis)
    knvis=E.SubElement(kn,"visual")
    E.SubElement(knvis,"origin",xyz="0 0 -0.008")
    E.SubElement(E.SubElement(knvis,"geometry"),"mesh",
                 filename=PACK+"17_front_knuckle_"+D+"{side}.stl",scale=SCALE)
    E.SubElement(knvis,"material",name="metal")
    for component in root.findall("./"+NS+"mounted_box"):
        name=component.get("name")
        if name=="towerpro_steering_servo":component.set("pz","0.029")
        elif name=="battery_pack":component.set("px","-0.005")
        elif name=="gearbox_differential":
            component.set("py","0.026")
            component.set("pz","0.026")
        elif name=="front_lidar":
            component.set("px","0.080")
    # Replace the provisional rectangular motor with a physical cylindrical
    # Ø31mm x 61mm motor, leaving the approximate motor mass unchanged.
    old_motor=root.find("./"+NS+"mounted_box[@name='bldc_motor']")
    if old_motor is not None:root.remove(old_motor)
    motor_link=root.find("./link[@name='bldc_motor']")
    if motor_link is None:motor_link=E.SubElement(root,"link",name="bldc_motor")
    else:
        for element in list(motor_link):motor_link.remove(element)
    inertial=E.SubElement(motor_link,"inertial")
    E.SubElement(inertial,"mass",value="0.18")
    E.SubElement(inertial,"inertia",
                 ixx="0.0000216225",ixy="0",ixz="0",
                 iyy="0.00006661125",iyz="0",izz="0.00006661125")
    visual=E.SubElement(motor_link,"visual")
    E.SubElement(visual,"origin",rpy="0 1.57079632679 0")
    c=E.SubElement(E.SubElement(visual,"geometry"),"cylinder")
    E.SubElement(c,"radius",value="0.0155")
    E.SubElement(c,"length",value="0.061")
    E.SubElement(visual,"material",name="metal")
    col=E.SubElement(motor_link,"collision")
    E.SubElement(col,"origin",rpy="0 1.57079632679 0")
    c=E.SubElement(E.SubElement(col,"geometry"),"cylinder")
    E.SubElement(c,"radius",value="0.0155")
    E.SubElement(c,"length",value="0.061")
    joint=root.find("./joint[@name='bldc_motor_mount']")
    if joint is None:joint=E.SubElement(root,"joint",name="bldc_motor_mount",type="fixed")
    for element in list(joint):joint.remove(element)
    E.SubElement(joint,"parent",link="base_link")
    E.SubElement(joint,"child",link="bldc_motor")
    E.SubElement(joint,"origin",xyz="-0.090 -0.026 0.024")
    tree.write(str(URDF_PATH),encoding="UTF-8",xml_declaration=True,pretty_print=True)


if __name__=="__main__":
    update_sdf()
    update_urdf()
    print("SDF/Xacro linked to the same printable STL mesh geometry.")
