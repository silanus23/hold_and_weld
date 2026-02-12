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

"""URDF-based seam generator - auto-detects seams from geometry.

Geometry-driven seam generation without joint type classification.
Generates weld trajectories from URDF analysis with configurable parameters.
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
    load_urdf_config,
)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Generate weld seams using URDF geometry',
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
    """Run URDF-based seam generation workflow."""
    args = parse_arguments()

    if args.input is None:
        print('ERROR: No input file specified and default config not found')
        print('Use --input to specify a YAML configuration file')
        return 1

    try:
        if args.verbose:
            print(f'Loading configuration from: {args.input}')

        seams, parameters, workpiece_config = load_urdf_config(args.input)

        if args.verbose:
            print(f'  Job: {Path(args.input).stem}')
            print(f'  Points per seam: {parameters["num_points"]}')
            print(f'  Work angle: {parameters["work_angle_deg"]}°')
            print(f'  Travel angle: {parameters["travel_angle_deg"]}°')
            print(f'  Gap: {parameters["gap_mm"]}mm')
            print()

        main_urdf = workpiece_config['main_part']['urdf_path']
        secondary_urdf = workpiece_config['secondary_part']['urdf_path']

        main_world_pose = workpiece_config['main_part'].get('world_pose')
        secondary_world_pose = workpiece_config['secondary_part'].get('world_pose')

        planner = URDFSeamPlanner(
            main_urdf_path=main_urdf,
            secondary_urdf_path=secondary_urdf,
            main_world_pose=main_world_pose,
            secondary_world_pose=secondary_world_pose
        )

        main_link = workpiece_config['main_part']['link_name']
        secondary_link = workpiece_config['secondary_part']['link_name']

        if args.verbose:
            print(f'  Main part URDF: {main_urdf}')
            print(f'  Main link: {main_link}')
            print(f'  Secondary part URDF: {secondary_urdf}')
            print(f'  Secondary link: {secondary_link}')
            print()
            print('Auto-detecting seams from geometry...')

        generated_seams = planner.auto_detect_and_generate_seams(
            main_link=main_link,
            secondary_link=secondary_link,
            parameters=parameters,
            min_seam_length_m=0.01
        )

        if not generated_seams:
            print('ERROR: No seams detected from geometry')
            return 1

        if args.verbose:
            print(f'  Detected {len(generated_seams)} seam segment(s)')

            num_edge = sum(1 for s in generated_seams if s.config.get('is_edge_joint', False))
            num_flat = len(generated_seams) - num_edge

            if num_edge > 0:
                print(f'  Edge joints (edge-on-edge): {num_edge}')
            if num_flat > 0:
                print(f'  Flat joints (edge-on-surface): {num_flat}')

            for idx, seam in enumerate(generated_seams):
                seam_length_mm = seam.line_segment.length() * 1000
                num_poses = len(seam.poses) if seam.poses else 0
                joint_type = 'EDGE' if seam.config.get('is_edge_joint') else 'FLAT'
                print(f'  Segment {idx}: {seam_length_mm:.1f}mm with {num_poses} poses [{joint_type}]')
            print()

        print(f'Generated {len(generated_seams)} seam segment(s)')
        print()

        if args.output is None:
            output_path = auto_generate_output_path(args.input)
            if args.verbose:
                print(f'Auto-generated output path: {output_path}')
        else:
            output_path = Path(args.output)

        metadata = {
            'input_file': str(Path(args.input).resolve()),
            'work_angle_deg': parameters['work_angle_deg'],
            'travel_angle_deg': parameters['travel_angle_deg'],
            'gap_mm': parameters['gap_mm'],
            'num_segments': len(generated_seams),
            'geometry_driven': True,
            'note': 'Seams split at surface boundaries, edge vs flat determined by boundary positions'
        }

        output_data = {
            'metadata': metadata,
            'seams': {}
        }

        for idx, seam in enumerate(generated_seams):
            seam_id = f'segment_{idx:03d}'

            seam.is_generated = True
            seam_dict = seam.to_dict()

            seam_dict['is_edge_joint'] = bool(seam.config.get('is_edge_joint', False))
            seam_dict['on_surface'] = str(seam.config.get('on_surface', 'unknown'))

            output_data['seams'][seam_id] = seam_dict

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            json.dump(output_data, f, indent=2)

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
