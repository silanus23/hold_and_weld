# Magic Wand - Quick Start Guide

## Overview
The magic wand system spawns the child_link at its end_pose location and visualizes surfaces from the latest JSON file in the `generated/` folder as interactive markers in RViz2.

## What It Does

1. **Spawns child_link at end_pose**: Reads the `end_pose` configuration from `objects.yaml` and spawns the child_link collision object at that position and orientation in the MoveIt planning scene.

2. **Visualizes JSON surfaces**: Automatically finds the latest JSON file in the `generated/` folder and creates interactive markers for each surface defined in the JSON.

## Files Created

- **Script**: `hold_and_weld_application/scripts/magic_wand.py`
  - Main Python script that handles spawning and visualization
  
- **Launch File**: `hold_and_weld_application/launch/magic_wand.launch.py`
  - Launch file to start the magic wand node

## Configuration

The magic wand reads from `hold_and_weld_application/config/collision_objects/objects.yaml`:

```yaml
child_link:
  urdf_path: 'urdf/environment/cube.urdf.xacro'
  id: 'child_link'
  spawn_name: 'child_link'
  pose:
    x: 1.2
    y: 0.3
    z: 0.125
  end_pose:
    position:
      x: 1.2
      y: -0.5
      z: 0.935
    orientation:
      x: 0.0
      y: 0.7071068
      z: 0.0
      w: 0.7071068
```

## Usage

### Build the Package

```bash
cd ~/ros2_yaskawa
colcon build --packages-select hold_and_weld_application
source install/setup.bash
```

### Launch the Magic Wand

```bash
ros2 launch hold_and_weld_application magic_wand.launch.py
```

### With Simulation Time

```bash
ros2 launch hold_and_weld_application magic_wand.launch.py use_sim_time:=true
```

## Features

### Interactive Markers
- Each surface from the JSON is rendered as an interactive marker
- Surface outline shown as line strips
- Normal vector shown as an arrow
- Markers are movable in the XY plane
- Colors match the part colors defined in the JSON

### Collision Object
- child_link is spawned at the exact end_pose position and orientation
- Automatically added to the MoveIt planning scene
- Collision geometry parsed from URDF

## JSON Format Expected

The script expects JSON files with this structure:

```json
{
  "metadata": {
    "version": "1.0",
    "timestamp": "2026-02-03T13:35:20.764158"
  },
  "parts": [
    {
      "name": "cube",
      "link_name": "child_link",
      "color": "#FF0000",
      "surfaces": [
        {
          "id": "child_link:0:top",
          "type": "planar",
          "center": [0.0, 0.0, 0.125],
          "normal": [0.0, 0.0, 1.0],
          "bounds": [0.25, 0.25],
          "corners": [[...], [...], ...]
        }
      ]
    }
  ]
}
```

## Integration with Existing System

The magic wand can be launched alongside your main system. For example:

```bash
# Terminal 1: Launch main system
ros2 launch hold_and_weld_application system.launch.py

# Terminal 2: Launch magic wand
ros2 launch hold_and_weld_application magic_wand.launch.py use_sim_time:=true
```

## Troubleshooting

- **No JSON files found**: Make sure you have JSON files in the `generated/` folder
- **Child link not spawning**: Check that the end_pose is properly defined in objects.yaml
- **No markers visible**: Open RViz2 and ensure interactive markers are enabled
- **Collision object subscribers**: The script waits for collision_object topic subscribers (MoveIt should be running)

## Next Steps

You can extend the magic wand to:
- Support different collision shapes (cylinder, sphere)
- Add custom marker controls
- Save marker positions back to JSON
- Generate welding paths from selected surfaces
