"""YAML configuration loader.

This module provides functions for loading YAML configuration files
containing servo positions and LAB colour thresholds.  The default
file paths point to resources in the project root.  If you move
configuration files to another location you can override these
paths accordingly.
"""

import os
import yaml

# Determine the project root (two directories up from this file)
_pkg_dir = os.path.dirname(__file__)
_project_root = os.path.abspath(os.path.join(_pkg_dir, '..'))

# Paths to the servo and colour configuration YAML files.  The
# original HiWonder SDK stores these at `/home/pi/TonyPi/lab_config.yaml`
# and `/home/pi/TonyPi/servo_config.yaml`, but here we default to
# local copies shipped with this repository.
lab_file_path = os.path.join(_project_root, 'lab_config.yaml')
servo_file_path = os.path.join(_project_root, 'servo_config.yaml')


def get_yaml_data(file_path: str):
    """Load a YAML file and return the parsed data.

    Args:
        file_path (str): Path to the YAML file.

    Returns:
        dict: Parsed YAML data, or an empty dict if loading fails.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data
    except Exception as e:
        print(f"Failed to load YAML file {file_path}: {e}")
        return {}