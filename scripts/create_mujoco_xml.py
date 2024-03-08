import xml.etree.ElementTree as ET
import argparse
import pathlib
from typing import List


def compose_robot_xml(
    xml_dir: pathlib.Path,
    spawn_z: float,
    imu_pos_str: str,
    imu_site_name: str,
    input_filename: str,
    output_filename: str,
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
        diffuse=".8 .8 .8",
        specular="0.3 0.3 0.3",
        pos="0 -4 4",
        cutoff="100",
    )

    # Add includes for other files
    for include in file_includes:
        print("Including xml from:", include)
        include_obj = ET.parse(xml_dir / include).getroot()[0]
        root.insert(0, include_obj)

    # Write the modified tree to a new file
    print("Writing to:", xml_dir / output_filename)
    tree.write(xml_dir / output_filename)


parser = argparse.ArgumentParser()
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

compose_robot_xml(
    xml_dir=args.xml_dir,
    spawn_z=args.spawn_z,
    imu_pos_str=args.imu_pos_str,
    imu_site_name=args.imu_site_name,
    file_includes=[
        "defaults.xml",
        "pupper_v3_actuators.xml",
        "sensors.xml",
        "assets.xml",
    ],
    input_filename=args.input_filename,
    output_filename=args.output_filename,
)
