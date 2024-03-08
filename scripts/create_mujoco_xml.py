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
        target="base_link",
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
        target="base_link",
        pos="0.5 -0.5 0.5",
    )

    # Add includes for other files
    for include in file_includes:
        print("Including xml from:", include)
        include_obj = ET.parse(xml_dir / include).getroot()
        for obj in include_obj:
            root.insert(0, obj)

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

    # Fixed base
    if fixed_base:
        base_joint = base_link.find("joint")
        base_link.remove(base_joint)
        output_path = output_path.with_suffix(".fixed_base.xml")

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
args = parser.parse_args()

if args.all_models:
    for mjx, fixed in itertools.product([False, True], repeat=2):
        compose_robot_xml(
            mjx_compatible=mjx,
            fixed_base=fixed,
            xml_dir=args.xml_dir,
            spawn_z=args.spawn_z,
            imu_pos_str=args.imu_pos_str,
            imu_site_name=args.imu_site_name,
            file_includes=[
                "defaults.xml",
                "actuators.xml",
                "sensors.xml",
                "assets.xml",
            ],
            input_filename=args.input_filename,
            output_filename=args.output_filename,
        )

else:
    compose_robot_xml(
        mjx_compatible=args.mjx,
        fixed_base=args.fixed_base,
        xml_dir=args.xml_dir,
        spawn_z=args.spawn_z,
        imu_pos_str=args.imu_pos_str,
        imu_site_name=args.imu_site_name,
        file_includes=[
            "defaults.xml",
            "actuators.xml",
            "sensors.xml",
            "assets.xml",
        ],
        input_filename=args.input_filename,
        output_filename=args.output_filename,
    )
