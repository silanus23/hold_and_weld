# Copyright 2025 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""File I/O utilities for loading YAML configs and exporting JSON poses."""

import json
import yaml
from pathlib import Path
from datetime import datetime
from .seam import Seam


def load_weld_job(yaml_path):
    """
    Load weld job configuration from a YAML file.

    Parameters
    ----------
    yaml_path : str
        Path to the YAML configuration file.

    Returns
    -------
    tuple
        A tuple containing the following elements:
        - seams : list
            List of Seam objects.
        - parameters : dict
            Dictionary of weld parameters.
        - surface_info : dict
            Dictionary containing surface geometry information.

    Raises
    ------
    FileNotFoundError
        If the YAML file does not exist.
    ValueError
        If the YAML structure is invalid.

    """
    yaml_path = Path(yaml_path)

    if not yaml_path.exists():
        raise FileNotFoundError(f"Config file not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    required_keys = ['seams', 'parameters', 'surface']
    for key in required_keys:
        if key not in config:
            raise ValueError(f"Missing required key in YAML: '{key}'")

    required_params = ['joint_type', 'work_angle_deg', 'travel_angle_deg', 'gap_mm', 'num_points']
    for param in required_params:
        if param not in config['parameters']:
            raise ValueError(f"Missing required parameter: '{param}'")

    if 'center' not in config['surface'] or 'normal' not in config['surface']:
        raise ValueError("Surface must have 'center' and 'normal' keys")

    if not config['seams']:
        raise ValueError("No seams defined in configuration")

    seams = []
    for i, seam_dict in enumerate(config['seams']):
        if 'start' not in seam_dict or 'end' not in seam_dict:
            raise ValueError(f"Seam {i} missing 'start' or 'end'")

        seams.append(Seam(seam_dict))

    return seams, config['parameters'], config['surface']


def export_to_json(seams, output_path, metadata=None):
    """
    Export generated seam poses to a JSON file.

    Parameters
    ----------
    seams : list
        List of Seam objects. Poses must be generated before export.
    output_path : str
        Path where the JSON file will be written.
    metadata : dict, optional
        Optional metadata to include in the output file.

    Raises
    ------
    RuntimeError
        If any seam has not been generated yet.

    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    for i, seam in enumerate(seams):
        if not seam.is_generated:
            raise RuntimeError(f"Seam {i} has not been generated yet - cannot export")

    data = {
        'metadata': {
            'generated_at': datetime.now().isoformat(),
            'num_seams': len(seams),
            'total_poses': sum(len(seam.poses) for seam in seams)
        },
        'seams': {}
    }

    if metadata:
        data['metadata'].update(metadata)

    for i, seam in enumerate(seams):
        data['seams'][f'seam_{i}'] = seam.to_dict()

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)


def auto_generate_output_path(input_path):
    """
    Generate an output path with a timestamp relative to the project root.

    The output directory will be:
    .../hold_and_weld_planning/generated/
    (i.e., one level up from the package source directory)
    """
    input_path = Path(input_path)
    job_name = input_path.stem
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    project_root = Path(__file__).parent.parent

    output_dir = project_root / "generated"
    output_dir.mkdir(parents=True, exist_ok=True)

    output_filename = f"{job_name}_{timestamp}.json"
    return output_dir / output_filename
