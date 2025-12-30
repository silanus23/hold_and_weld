#!/usr/bin/env python3

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

"""URDF-based seam generator - auto-detects surfaces and joint types from geometry."""

import argparse
from pathlib import Path
import sys

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).parent))

from hold_and_weld_planning.urdf import URDFSeamPlanner
from hold_and_weld_planning.utils import (
    auto_generate_output_path,
    export_to_json,
    load_urdf_config,
)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Generate weld seams using URDF geometry (auto-detect surfaces)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default config
  %(prog)s

  # Specify input config
  %(prog)s --input urdf_weld_config.yaml

  # Verbose output
  %(prog)s --input urdf_weld_config.yaml --verbose
        """
    )

    default_config = (
        Path(__file__).parent.parent / 'config' / 'urdf_welding_conf.yaml'
    )

    parser.add_argument(
        '--input', '-i',
        type=str,
        default=str(default_config) if default_config.exists() else None,
        help='Input YAML configuration file'
    )

    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output JSON file (default: auto-generated)'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed information'
    )

    return parser.parse_args()


def main():
    """Orchestrate URDF-based seam generation."""
    args = parse_arguments()

    if args.input is None:
        print('ERROR: No input file specified and default config not found')
        print('Use --input to specify a YAML configuration file')
        return 1

    try:
        if args.verbose:
            print(f'Loading URDF-based configuration from: {args.input}')

        seams, parameters, workpiece_config = load_urdf_config(args.input)

        if args.verbose:
            print(f'  Job: {Path(args.input).stem}')
            print(f'  Number of seams: {len(seams)}')
            print(f'  Points per seam: {parameters["num_points"]}')
            print(f'  Work angle: {parameters["work_angle_deg"]}°')
            print(f'  Travel angle: {parameters["travel_angle_deg"]}°')
            print(f'  Gap: {parameters["gap_mm"]}mm')
            print()

        planner = URDFSeamPlanner(workpiece_config)

        main_link = workpiece_config['main_part']['link_name']
        extra_link = workpiece_config['extra_part']['link_name']

        if args.verbose:
            print(
                f'  Main part URDF: '
                f'{workpiece_config["main_part"]["urdf_path"]}'
            )
            print(f'  Main link: {main_link}')
            print(
                f'  Extra part URDF: '
                f'{workpiece_config["extra_part"]["urdf_path"]}'
            )
            print(f'  Extra link: {extra_link}')
            print()

        joint_type, surface_info, success = planner.generate_seams(
            seams, parameters
        )

        if not success:
            print('ERROR: Failed to generate seams')
            return 1

        if args.verbose:
            print(f'  Detected joint type: {joint_type}')
            print(f'  Surface center: {surface_info["center"]}')
            print(f'  Surface normal: {surface_info["normal"]}')
            print()

        print(f'Generated {len(seams)} seam(s) with joint type: {joint_type}')
        print()

        if args.output is None:
            output_path = auto_generate_output_path(args.input)
            if args.verbose:
                print(f'Auto-generated output path: {output_path}')
        else:
            output_path = Path(args.output)

        metadata = {
            'input_file': str(Path(args.input).resolve()),
            'joint_type': joint_type,
            'joint_type_auto_detected': True,
            'work_angle_deg': parameters['work_angle_deg'],
            'travel_angle_deg': parameters['travel_angle_deg'],
            'gap_mm': parameters['gap_mm'],
            'surface_center': surface_info['center'],
            'surface_normal': surface_info['normal']
        }

        export_to_json(seams, output_path, metadata)

        print(f'Exported to: {output_path}')

        return 0

    except FileNotFoundError as e:
        print(f'ERROR: {e}')
        return 1
    except ValueError as e:
        print(f'ERROR: Invalid configuration - {e}')
        return 1
    except Exception as e:
        print(f'ERROR: {e}')
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
