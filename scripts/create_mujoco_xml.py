import xml.etree.ElementTree as ET
import argparse
import pathlib
from typing import List
import itertools


def compose_robot_xml(
    xml_dir: pathlib.Path,
    spawn_z: float,
    imu_pos_str: str,
    imu_site_name: str,
    input_filename: str,
    output_filename: str,
    mjx_compatible: bool,
    fixed_base: bool,
    lower_leg_names: List[str],
    position_control: bool,
    position_control_kp: float = 0.0,
    position_control_kd: float = 0.0,
    file_includes: List[str] = [],
):
    # Parse the URDF file
    print("Reading input xml:", xml_dir / input_filename)
    tree = ET.parse(xml_dir / input_filename)
    root = tree.getroot()

    # Remove existing default tag
    for child in list(root):
        if child.tag == "default":
            root.remove(child)

    # Iterate over all 'geom' elements and remove 'contype' and 'conaffinity' attributes
    # Remove all contype and conaffinity tags so everything becomes collision geom
    # TODO buggy: too many self collisions
    # for geom in root.findall(".//geom"):
    #     if "contype" in geom.attrib:
    #         del geom.attrib["contype"]
    #     if "conaffinity" in geom.attrib:
    #         del geom.attrib["conaffinity"]

    worldbody = root.find("worldbody")

    base_link = worldbody.find("body")
    base_link.set("pos", f"0 0 {spawn_z}")
    ET.SubElement(
        base_link,
        "site",
        name=imu_site_name,
        pos=imu_pos_str,
    )

    if not fixed_base:
        # Add floor
        ET.SubElement(
            worldbody,
            "geom",
            name="floor",
            size="0 0 .05",
            type="plane",
            material="grid",
            condim="3",
        )

        # Add second visual floor
        ET.SubElement(
            worldbody,
            "geom",
            name="floor_visual",
            size="0 0 .05",
            pos="0 0 -0.001",
            type="plane",
            material="grid",
            condim="3",
            contype="0",
            conaffinity="0",
            group="1",
        )

    # Add light
    ET.SubElement(
        worldbody,
        "light",
        name="spotlight",
        mode="targetbodycom",
        target="leg_front_r_1" if mjx_compatible and fixed_base else "base_link",
        diffuse="0.9 0.9 0.9",
        specular="0.3 0.3 0.3",
        pos="0 -4 4",
        cutoff="100",
    )

    # Add camera
    ET.SubElement(
        worldbody,
        "camera",
        name="tracking_cam",
        mode="targetbody",
        target="leg_front_r_1" if mjx_compatible and fixed_base else "base_link",
        pos="0.5 -0.5 0.5",
    )

    # Add includes for other files
    for include in file_includes:
        print("Including xml from:", include)
        include_obj = ET.parse(xml_dir / include).getroot()
        for obj in include_obj:
            root.insert(0, obj)

    # Add feet sites necessary for RL
    for lower_leg_name in lower_leg_names:
        lower_leg_elem = tree.find(f".//body[@name='{lower_leg_name}']")
        foot_elem = lower_leg_elem.find(".//geom")
        foot_pos = foot_elem.get("pos")
        ET.SubElement(
            lower_leg_elem, "site", name=lower_leg_name + "_foot_site", pos=foot_pos
        )

    output_path = pathlib.Path(xml_dir / output_filename)

    # MJX compatibility
    if mjx_compatible:
        # Replace cylinders with sphere
        cylinders = root.findall(".//geom[@type='cylinder']")
        for cyl in cylinders:
            # Change type to "sphere"
            cyl.set("type", "sphere")

            # Assume the cylinder's size attribute format is "radius length" (e.g., "0.025 0.015")
            radius = cyl.get("size").split(" ")[0]
            cyl.set("size", radius)

        # Set condim to 3 and use only linear friction
        default_section = root.find(".//default")
        default_geom = default_section.find("geom")
        default_geom.set("condim", "3")
        linear_friction = default_geom.get("friction").split(" ")[0]
        default_geom.set("friction", linear_friction)

        # change friction from elliptical to pyramidal
        option = root.find(".//option")
        option.set("cone", "pyramidal")

        # remove frictionloss field
        default_joint = default_section.find("joint")
        del default_joint.attrib["frictionloss"]

        # save as .mjx.xml for mjx compatible xml
        output_path = output_path.with_suffix(".mjx.xml")

    if position_control:
        default_section = root.find(".//default")
        general_actuator = default_section.find("general")
        general_actuator.set("biastype", "affine")
        if position_control:
            general_actuator.set("gainprm", f"{position_control_kp} 0 0")
            general_actuator.set(
                "biasprm", f"0 -{position_control_kp} -{position_control_kd}"
            )
        output_path = output_path.with_suffix(".position.xml")

    # Fixed base
    if fixed_base:
        # remove free joint
        base_joint = base_link.find("joint")
        base_link.remove(base_joint)
        output_path = output_path.with_suffix(".fixed_base.xml")

        # remove base_link inertia for mjx model
        if mjx:
            base_inertia = base_link.find("inertial")
            base_link.remove(base_inertia)

    print("Writing to:", output_path)
    tree.write(output_path)


parser = argparse.ArgumentParser()
parser.add_argument(
    "--mjx",
    action="store_true",
    help="Use MJX-compatible settings like no cylindrical geometry, inly condim=3, etc",
)
parser.add_argument(
    "--fixed_base",
    action="store_true",
    help="Fix the body in space",
)
parser.add_argument(
    "--all_models",
    action="store_true",
    help="Generate all combinations of mjx and fixed base models",
)
parser.add_argument("--xml_dir", type=pathlib.Path, required=True, help="directory")
parser.add_argument(
    "--input_filename",
    type=str,
    default="exported_mjmodel.xml",
    help="filename of input mujoco xml file. Must be inside xml_dir",
)
parser.add_argument(
    "--output_filename",
    type=str,
    default="pupper_v3_complete.xml",
    help="filename of output mujoco xml file. Will be saved inside xml_dir",
)
parser.add_argument(
    "--spawn_z", type=float, help="Height (m) at which to spawn the body", default=0.13
)
parser.add_argument(
    "--imu_pos_str",
    type=str,
    help="xyz coordinates (m) of IMU in body frame",
    default="0.09 0 0.032",
)
parser.add_argument(
    "--imu_site_name", type=str, default="body_imu_site", help="Name of IMU site"
)
parser.add_argument(
    "--lower_leg_names",
    nargs="+",
    help="Names of lower legs",
    default=["leg_front_r_3", "leg_front_l_3", "leg_back_r_3", "leg_back_l_3"],
)
parser.add_argument(
    "--position_control",
    action="store_true",
    help="Specify to make actuators position-control",
)
parser.add_argument(
    "--position_control_kp",
    type=float,
    default=5.0,
    help="If position control is specified, used to set actuator kp",
)
parser.add_argument(
    "--position_control_kd",
    type=float,
    default=0.1,
    help="If position control is specified, used to set actuator kd",
)
args = parser.parse_args()

common_args = dict(
    xml_dir=args.xml_dir,
    spawn_z=args.spawn_z,
    imu_pos_str=args.imu_pos_str,
    imu_site_name=args.imu_site_name,
    file_includes=[
        "defaults.xml",
        "actuators.xml",
        "sensors.xml",
        "assets.xml",
        "home_keyframe.xml",
    ],
    input_filename=args.input_filename,
    output_filename=args.output_filename,
    lower_leg_names=args.lower_leg_names,
    position_control_kp=args.position_control_kp,
    position_control_kd=args.position_control_kd,
)

if args.all_models:
    for mjx, fixed, position_control in itertools.product([False, True], repeat=3):
        compose_robot_xml(
            mjx_compatible=mjx,
            fixed_base=fixed,
            position_control=position_control,
            **common_args,
        )

else:
    compose_robot_xml(
        mjx_compatible=args.mjx,
        fixed_base=args.fixed_base,
        position_control=args.position_control,
        **common_args,
    )
