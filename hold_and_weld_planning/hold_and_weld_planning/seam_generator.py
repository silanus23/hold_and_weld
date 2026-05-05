#!/usr/bin/env python3

# Copyright 2026 Berkan Tali
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

"""Command-line interface for automated weld seam generation from URDF/CAD geometry."""

import argparse
import logging
from pathlib import Path
import sys

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).parent))

from hold_and_weld_planning.planning.job_planner import JobPlanner
from hold_and_weld_planning.utils.path_utils import (
    auto_generate_output_path,
    export_to_json,
    load_urdf_config,
)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Generate weld seams from URDF geometry',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default config
  %(prog)s

  # Specify input config
  %(prog)s --input urdf_weld_config.yaml

  # Verbose output
  %(prog)s --input urdf_weld_config.yaml --verbose
        """,
    )

    default_config = Path(__file__).parent.parent / 'config' / 'urdf_welding_conf.yaml'

    parser.add_argument(
        '--input',
        '-i',
        type=str,
        default=str(default_config) if default_config.exists() else None,
        help='Input YAML configuration file',
    )

    parser.add_argument(
        '--output',
        '-o',
        type=str,
        default=None,
        help='Output JSON file (default: auto-generated)',
    )

    parser.add_argument(
        '--verbose', '-v', action='store_true', help='Print detailed information'
    )

    return parser.parse_args()


def setup_logging(verbose: bool) -> None:
    """Configure logging based on verbosity level."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(levelname)s - %(name)s - %(message)s',
        force=True,
    )


def main():
    """Run seam generation workflow from CLI."""
    args = parse_arguments()

    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    if args.input is None:
        logger.error('No input file specified and default config not found')
        print('Use --input to specify a YAML configuration file')
        return 1

    try:
        if args.verbose:
            print(f'Loading configuration from: {args.input}')

        seams, parameters, workpiece_config = load_urdf_config(args.input)
        main_path = workpiece_config['main_part']['main_path']
        secondary_path = workpiece_config['secondary_part']['secondary_path']

        main_world_pose = workpiece_config['main_part'].get('world_pose')
        secondary_world_pose = workpiece_config['secondary_part'].get('world_pose')

        # Extract mode from workpiece config
        mode = workpiece_config.get('mode', 'auto')

        if args.verbose:
            print(f'  Job: {Path(args.input).stem}')
            print(f'  Points per seam: {parameters.get("num_points", 100)}')
            print(f'  Work angle: {parameters["work_angle_deg"]}')
            print(f'  Travel angle: {parameters["travel_angle_deg"]}')
            print(f'  Gap: {parameters["gap_mm"]}mm')
            print()
            print(f'  Main part URDF: {main_path}')
            print(f'  Secondary part URDF: {secondary_path}')
            print(f'  Mode: {mode}')
            print()

        planner = JobPlanner(
            main_path=main_path,
            secondary_path=secondary_path,
            main_world_pose=main_world_pose,
            secondary_world_pose=secondary_world_pose,
            parameters=parameters,
            mode=mode,
        )

        logger.info('Starting weld job planning')

        generated_seams = planner.plan_job()

        if not generated_seams:
            logger.error('No seams generated')
            return 1

        if args.verbose:
            num_edge = sum(
                1 for s in generated_seams if s.config.get('is_edge_joint', False)
            )
            num_flat = len(generated_seams) - num_edge

            if num_edge > 0:
                print(f'  Edge joints (edge-on-edge): {num_edge}')
            if num_flat > 0:
                print(f'  Flat joints (edge-on-surface): {num_flat}')
            print()

            for idx, seam in enumerate(generated_seams):
                seam_length_mm = seam.length() * 1000
                num_poses = len(seam.poses) if seam.poses else 0
                joint_type = 'EDGE' if seam.config.get('is_edge_joint') else 'FLAT'
                seg_type = seam.segment_type.upper()
                print(
                    f'  Seam {idx}: {seg_type}, {seam_length_mm:.1f}mm, '
                    f'{num_poses} poses [{joint_type}]'
                )
            print()

        logger.info(f'Successfully generated {len(generated_seams)} seam(s)')

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
            'pipeline': mode,
        }

        export_to_json(generated_seams, output_path, metadata)
        logger.info(f'Exported to: {output_path}')
        return 0

    except FileNotFoundError as e:
        logger.error(f'{e}')
        return 1
    except ValueError as e:
        logger.error(f'Invalid configuration - {e}')
        return 1
    except Exception as e:
        logger.error(f'Unexpected error: {e}', exc_info=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
