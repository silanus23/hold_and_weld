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
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from ..core.seam import Seam

logger = logging.getLogger(__name__)


def load_urdf_config(
    yaml_path: str | Path,
) -> Tuple[List[Seam], Dict[str, Any], Dict[str, Any]]:
    """Load URDF-based weld configuration from YAML.

    Args:
        yaml_path: Path to YAML config file

    Returns:
        Tuple of (seams, parameters, workpiece_config)

    Raises:
        FileNotFoundError: If config file not found
        ValueError: If config structure invalid
    """
    yaml_path = Path(yaml_path)

    if not yaml_path.exists():
        raise FileNotFoundError(f'Config file not found: {yaml_path}')

    logger.info(f'Loading URDF-based config from: {yaml_path}')

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    if 'workpiece' not in config:
        raise ValueError(f"Config missing 'workpiece' section in {yaml_path}")

    if (
        'main_part' not in config['workpiece']
        or 'secondary_part' not in config['workpiece']
    ):
        raise ValueError(f"Config must have both 'main_part' and 'secondary_part' in {yaml_path}")

    if 'main_path' not in config['workpiece']['main_part']:
        raise ValueError(f"Missing 'main_path' under workpiece.main_part in {yaml_path}")

    if 'secondary_path' not in config['workpiece']['secondary_part']:
        raise ValueError(f"Missing 'secondary_path' under workpiece.secondary_part in {yaml_path}")

    if 'parameters' not in config:
        raise ValueError(f"Config missing 'parameters' section in {yaml_path}")

    auto_detect = config['workpiece'].get('auto_detect_seams', False)

    # Only require seams if auto-detect is disabled
    seams = []
    if not auto_detect:
        if 'seams' not in config or not config['seams']:
            raise ValueError(f"Config missing 'seams' or seams list is empty in {yaml_path}")

        for seam_dict in config['seams']:
            if 'start' not in seam_dict or 'end' not in seam_dict:
                raise ValueError(f"Each seam must have 'start' and 'end' in {yaml_path}")
            seams.append(Seam(seam_dict))

        logger.info(f'Loaded {len(seams)} seam(s) from config')
    else:
        logger.info('Auto-detect mode enabled, seams will be extracted from geometry')

    return seams, config['parameters'], config['workpiece']


def export_to_json(
    seams: List[Seam],
    output_path: str | Path,
    metadata: Optional[Dict[str, Any]] = None,
) -> None:
    """Export generated seam poses to JSON file.

    Args:
        seams: List of generated Seam objects
        output_path: Output file path
        metadata: Optional metadata dict

    Raises:
        RuntimeError: If any seam not generated yet
    """
    output_path = Path(output_path)

    for i, seam in enumerate(seams):
        if not seam.is_generated:
            raise RuntimeError(f'Seam {i} has not been generated yet - cannot export')

    logger.debug(f'Creating output directory: {output_path.parent}')
    output_path.parent.mkdir(parents=True, exist_ok=True)

    total_poses = sum(len(seam.poses) for seam in seams)

    data = {
        'metadata': {
            'generated_at': datetime.now().isoformat(),
            'num_seams': len(seams),
            'total_poses': total_poses,
        },
        'seams': {},
    }

    if metadata:
        data['metadata'].update(metadata)

    for i, seam in enumerate(seams):
        data['seams'][f'seam_{i}'] = seam.to_dict()

    logger.info(f'Exporting {len(seams)} seam(s) with {total_poses} total poses to: {output_path}')

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

    logger.info(f'Successfully exported to {output_path}')


def auto_generate_output_path(input_path: str | Path) -> Path:
    """Generate output path in hold_and_weld_application/trajectories/ directory."""
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
            logger.debug(f'Detected workspace root: {workspace_root}')
            break

    if workspace_root is None:
        # Fallback: try relative path from current location
        workspace_root = current_path.parent.parent.parent.parent.parent
        logger.warning(
            f'Could not detect workspace root, using fallback: {workspace_root}'
        )

    output_dir = (
        workspace_root
        / 'src'
        / 'hold_and_weld'
        / 'hold_and_weld_application'
        / 'trajectories'
    )

    logger.debug(f'Creating output directory: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)

    output_filename = f'{job_name}_{timestamp}.json'
    output_path = output_dir / output_filename

    logger.info(f'Generated output path: {output_path}')
    return output_path
