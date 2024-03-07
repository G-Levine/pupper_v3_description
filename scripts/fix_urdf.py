import xml.etree.ElementTree as ET
import argparse
import pathlib


def fix_mesh_paths(urdf_file_path, output_file_path):
    # Parse the URDF file
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    # Loop through all 'mesh' tags and fix the 'filename' attribute
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        # Check if the filename needs to be updated
        if filename.startswith("../"):
            # Update the filename
            new_filename = (
                "package://pupper_v3_description/description/" + filename[3:]
            ).replace(" ", "")
            mesh.set("filename", new_filename)

    # Write the modified tree to a new file
    tree.write(output_file_path)


parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path", type=pathlib.Path, required=True)
args = parser.parse_args()

# Example usage
urdf_file_path = "path/to/your/original/urdf/file.urdf"
output_file_path = "path/to/your/fixed/urdf/file.urdf"

fix_mesh_paths(args.urdf_path, args.urdf_path.with_suffix(".fixed.urdf"))
