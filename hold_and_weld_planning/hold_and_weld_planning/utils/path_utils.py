# Copyright 2025 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""File I/O utilities for loading YAML configs and exporting JSON poses."""

from datetime import datetime
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from ..core.seam import Seam


def load_weld_job(
    yaml_path: str | Path,
) -> Tuple[List[Seam], Dict[str, Any], Dict[str, Any]]:
    """Load weld job configuration from a YAML file.

    Args:
        yaml_path: Path to the YAML configuration file.

    Returns:
        A tuple containing the following elements:
        - seams: List of Seam objects.
        - parameters: Dictionary of weld parameters.
        - surface_info: Dictionary containing surface geometry information.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
        ValueError: If the YAML structure is invalid.
    """
    yaml_path = Path(yaml_path)

    if not yaml_path.exists():
        raise FileNotFoundError(f'Config file not found: {yaml_path}')

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    required_keys = ['seams', 'parameters', 'surface']
    for key in required_keys:
        if key not in config:
            raise ValueError(f"Missing required key in YAML: '{key}'")

    required_params = [
        'joint_type',
        'work_angle_deg',
        'travel_angle_deg',
        'gap_mm',
        'num_points',
    ]
    for param in required_params:
        if param not in config['parameters']:
            raise ValueError(f"Missing required parameter: '{param}'")

    if 'center' not in config['surface'] or 'normal' not in config['surface']:
        raise ValueError("Surface must have 'center' and 'normal' keys")

    if not config['seams']:
        raise ValueError('No seams defined in configuration')

    seams = []
    for i, seam_dict in enumerate(config['seams']):
        if 'start' not in seam_dict or 'end' not in seam_dict:
            raise ValueError(f"Seam {i} missing 'start' or 'end'")

        seams.append(Seam(seam_dict))

    return seams, config['parameters'], config['surface']


def load_urdf_config(
    yaml_path: str | Path,
) -> Tuple[List[Seam], Dict[str, Any], Dict[str, Any]]:
    """Load URDF-based weld configuration (URDF workflow).

    Args:
        yaml_path: Path to YAML config file.

    Returns:
        A tuple containing:
        - seams: List of Seam objects.
        - parameters: Dictionary of weld parameters.
        - workpiece_config: Dictionary with 'main_part' and 'secondary_part'.

    Raises:
        FileNotFoundError: If config file not found.
        ValueError: If config structure invalid.
    """
    yaml_path = Path(yaml_path)

    if not yaml_path.exists():
        raise FileNotFoundError(f'Config file not found: {yaml_path}')

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    if 'workpiece' not in config:
        raise ValueError("Config missing 'workpiece' section")

    if (
        'main_part' not in config['workpiece']
        or 'secondary_part' not in config['workpiece']
    ):
        raise ValueError("Config must have both 'main_part' and 'secondary_part'")

    if 'parameters' not in config:
        raise ValueError("Config missing 'parameters' section")

    # Check if auto-detect is enabled
    auto_detect = config['workpiece'].get('auto_detect_seams', False)

    # Only require seams if auto-detect is disabled
    seams = []
    if not auto_detect:
        if 'seams' not in config or not config['seams']:
            raise ValueError("Config missing 'seams' or seams list is empty")

        for seam_dict in config['seams']:
            if 'start' not in seam_dict or 'end' not in seam_dict:
                raise ValueError("Each seam must have 'start' and 'end'")
            seams.append(Seam(seam_dict))

    return seams, config['parameters'], config['workpiece']


def export_to_json(
    seams: List[Seam],
    output_path: str | Path,
    metadata: Optional[Dict[str, Any]] = None,
) -> None:
    """Export generated seam poses to a JSON file.

    Args:
        seams: List of Seam objects. Poses must be generated before export.
        output_path: Path where the JSON file will be written.
        metadata: Optional metadata to include in the output file.

    Raises:
        RuntimeError: If any seam has not been generated yet.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    for i, seam in enumerate(seams):
        if not seam.is_generated:
            raise RuntimeError(f'Seam {i} has not been generated yet - cannot export')

    data = {
        'metadata': {
            'generated_at': datetime.now().isoformat(),
            'num_seams': len(seams),
            'total_poses': sum(len(seam.poses) for seam in seams),
            'num_segments': len(seams),
            'pipeline': 'mesh-based geometry detection',
        },
        'seams': {},
    }

    if metadata:
        data['metadata'].update(metadata)

    for i, seam in enumerate(seams):
        data['seams'][f'seam_{i}'] = seam.to_dict()

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)


def auto_generate_output_path(input_path: str | Path) -> Path:
    """Generate output path in hold_and_weld_application/trajectories/.

    This allows C++ action server to read JSON without rebuilding.

    Output location:
        src/hold_and_weld/hold_and_weld_application/trajectories/

    Args:
        input_path: Path to the input file.

    Returns:
        Path object for the output file.
    """
    input_path = Path(input_path)
    job_name = input_path.stem
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Navigate to workspace root and then to the source directory
    # This works whether running from build/ or install/ directories
    current_path = Path(__file__).resolve()

    workspace_root = None
    for parent in current_path.parents:
        src_path = parent / 'src'
        if src_path.exists() and (src_path / 'hold_and_weld').exists():
            workspace_root = parent
            break

    if workspace_root is None:
        # Fallback: try relative path from current location
        workspace_root = current_path.parent.parent.parent.parent.parent

    output_dir = (
        workspace_root
        / 'src'
        / 'hold_and_weld'
        / 'hold_and_weld_application'
        / 'trajectories'
    )

    output_dir.mkdir(parents=True, exist_ok=True)

    output_filename = f'{job_name}_{timestamp}.json'
    return output_dir / output_filename
