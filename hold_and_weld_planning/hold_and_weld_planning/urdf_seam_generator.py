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

"""URDF-based seam generator - auto-detects surfaces and joint types from geometry.

This module provides a command-line interface for generating weld seam trajectories
by automatically analyzing URDF geometry files. It detects joint types (butt, lap,
T-joint, etc.) and generates appropriate seam paths with configurable welding parameters.
"""

import argparse
import json
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
    """Parse command line arguments for URDF-based seam generation.

    Returns:
        argparse.Namespace: Parsed arguments containing:
            - input: Path to input YAML configuration file
            - output: Optional path to output JSON file
            - verbose: Flag for detailed output
    """
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
    """Orchestrate URDF-based seam generation workflow.

    Loads URDF configuration, analyzes workpiece geometry to detect joint types,
    generates seam trajectories with specified welding parameters, and exports
    results to JSON format.

    Returns:
        int: Exit code (0 for success, 1 for error)
    """
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
        secondary_link = workpiece_config['secondary_part']['link_name']
        auto_detect = workpiece_config.get('auto_detect_seams', False)

        if args.verbose:
            print(
                f'  Main part URDF: '
                f'{workpiece_config["main_part"]["urdf_path"]}'
            )
            print(f'  Main link: {main_link}')
            print(
                f'  Secondary part URDF: '
                f'{workpiece_config["secondary_part"]["urdf_path"]}'
            )
            print(f'  Secondary link: {secondary_link}')
            print(f'  Auto-detect seams: {auto_detect}')
            print()

        # Use different workflow based on auto_detect_seams setting
        if auto_detect:
            if args.verbose:
                print('Auto-detecting seams from geometry...')

            seams_data, success = planner.auto_detect_and_generate_seams(
                parameters
            )

            if not success or not seams_data:
                print('ERROR: Failed to auto-detect or generate seams')
                return 1

            # Extract joint type from first seam (all should have same joint type)
            joint_type = seams_data[0]['joint_type']

            if args.verbose:
                print(f'  Detected {len(seams_data)} seam(s)')
                print(f'  Detected joint type: {joint_type}')
                for idx, seam_data in enumerate(seams_data):
                    seam_length_mm = seam_data['length'] * 1000
                    num_poses = len(seam_data['poses'])
                    print(
                        f'  Seam {idx}: {seam_length_mm:.1f}mm '
                        f'with {num_poses} poses'
                    )
                print()

            print(
                f'Auto-detected and generated {len(seams_data)} seam(s) '
                f'with joint type: {joint_type}'
            )
            print()

            # Build metadata from auto-detected data
            metadata = {
                'input_file': str(Path(args.input).resolve()),
                'joint_type': joint_type,
                'joint_type_auto_detected': True,
                'seams_auto_detected': True,
                'work_angle_deg': parameters['work_angle_deg'],
                'travel_angle_deg': parameters['travel_angle_deg'],
                'gap_mm': parameters['gap_mm']
            }

            if args.output is None:
                output_path = auto_generate_output_path(args.input)
                if args.verbose:
                    print(f'Auto-generated output path: {output_path}')
            else:
                output_path = Path(args.output)

            # Convert seams_data to JSON-compatible format
            output_data = {
                'metadata': metadata,
                'seams': {}
            }

            for seam_data in seams_data:
                seam_id = seam_data['seam_id']
                output_data['seams'][seam_id] = {
                    'start': seam_data['start'],
                    'end': seam_data['end'],
                    'length_m': seam_data['length'],
                    'poses': seam_data['poses'],
                    'num_poses': len(seam_data['poses']),
                    'joint_type': seam_data['joint_type'],
                    'main_surface_id': seam_data['main_surface_id'],
                    'secondary_surface_id': seam_data['secondary_surface_id']
                }

            # Write JSON file
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)

        else:
            # Manual seams workflow
            if not seams:
                print(
                    'ERROR: No seams specified and auto_detect_seams '
                    'is disabled'
                )
                return 1

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

            print(
                f'Generated {len(seams)} seam(s) with joint type: '
                f'{joint_type}'
            )
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
