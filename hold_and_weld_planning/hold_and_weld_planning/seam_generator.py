#!/usr/bin/env python3

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

"""Main user entry point - orchestrates loading, planning, and exporting."""

import sys
import argparse
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).parent.parent))

from hold_and_weld_planning import path_utils
from hold_and_weld_planning.weld_planner import WeldPlanner


def parse_arguments():
    """Parse command line arguments with smart defaults."""
    parser = argparse.ArgumentParser(
        description="Generate weld seam trajectories from YAML configuration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default config
  %(prog)s

  # Specify input config
  %(prog)s --input my_job.yaml

  # Specify both input and output
  %(prog)s --input my_job.yaml --output my_poses.json

  # Verbose output
  %(prog)s --input my_job.yaml --verbose
        """
    )

    default_config = Path(__file__).parent.parent / "config"/ "weld_config.yaml"

    parser.add_argument(
        '--input', '-i',
        type=str,
        default=str(default_config) if default_config.exists() else None,
        help='Input YAML configuration file (default: config/weld_config.yaml)'
    )

    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output JSON file (default: auto-generated in ~/.hold_and_weld/generated/)'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed information during generation'
    )

    return parser.parse_args()


def main():
    """Orchestrate loading, planning, and exporting of weld seams."""
    args = parse_arguments()

    if args.input is None:
        print("ERROR: No input file specified and default config not found")
        print("Use --input to specify a YAML configuration file")
        return 1

    try:
        # Load Configuration
        if args.verbose:
            print(f"Loading configuration from: {args.input}")

        seams, parameters, surface_info = path_utils.load_weld_job(args.input)

        if args.verbose:
            print(f"  Job: {Path(args.input).stem}")
            print(f"  Joint type: {parameters['joint_type']}")
            print(f"  Number of seams: {len(seams)}")
            print(f"  Points per seam: {parameters['num_points']}")
            print(f"  Work angle: {parameters['work_angle_deg']}°")
            print(f"  Travel angle: {parameters['travel_angle_deg']}°")
            print(f"  Gap: {parameters['gap_mm']}mm")
            print()

        # Create Planner
        planner = WeldPlanner(parameters)

        if args.verbose:
            print(f"Created planner for {parameters['joint_type']}")
            print()

        # Generate Poses for Each Seam
        if args.verbose:
            print(f"Generating poses for {len(seams)} seam(s)...")

        for i, seam in enumerate(seams):
            line = seam.line_segment

            if args.verbose:
                print(f"  Processing seam {i}:")
                print(f"    Start: [{line.start[0]:.3f}, {line.start[1]:.3f}, {line.start[2]:.3f}]")
                print(f"    End:   [{line.end[0]:.3f}, {line.end[1]:.3f}, {line.end[2]:.3f}]")
                print(f"    Length: {line.length()*1000:.1f}mm")

            success = planner.generate_seam(seam, surface_info)

            if not success:
                print(f"✗ Failed to generate seam {i}")
                return 1

            if args.verbose:
                print(f"Generated {len(seam.poses)} poses")

        print(f"Generated {len(seams)} seam(s)")
        print()

        # Export to JSON
        if args.output is None:
            output_path = path_utils.auto_generate_output_path(args.input)
            if args.verbose:
                print(f"Auto-generated output path: {output_path}")
        else:
            output_path = Path(args.output)

        metadata = {
            'input_file': str(Path(args.input).resolve()),
            'joint_type': parameters['joint_type'],
            'work_angle_deg': parameters['work_angle_deg'],
            'travel_angle_deg': parameters['travel_angle_deg'],
            'gap_mm': parameters['gap_mm']
        }

        path_utils.export_to_json(seams, output_path, metadata)

        total_poses = sum(len(seam.poses) for seam in seams)

        return 0

    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        return 1
    except ValueError as e:
        print(f"ERROR: Invalid configuration - {e}")
        return 1
    except Exception as e:
        print(f"ERROR: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
