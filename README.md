# Robot Arm Simulation with Box2D

3-link robot arm 2D simulation using Box2D physics engine with pygame visualization. The robot moves to target positions while avoiding obstacles.

## Project Structure

```
robot_sim_v1/
├── env.py           # Robot environment setup (world, links, obstacles)
├── policy.py        # Control policies (potential field, RMP)
├── render.py        # Visualization rendering
├── main.py          # Main simulation
└── record_video.py  # Video recording functionality
```

## Installation & Setup

### 1. Create Virtual Environment (using uv)
```bash
# Navigate to project directory
cd pybox
uv venv pybox2d
source pybox2d/bin/activate  # macOS/Linux
```

### 2. Install All Dependencies
```bash
# Install all required packages from pyproject.toml
uv pip install -e .

# For development (includes Jupyter support)
uv pip install -e ".[dev]"
```

## Usage

### 1. Basic Simulation

**Run with default settings (target: 5.0, 5.0):**
```bash
python main.py
```

**Set specific target position:**
```bash
python main.py --target 6.0 2.0
python main.py --target 4.0 3.0
python main.py --target 8.0 -1.0
```

### 2. Video Recording

**Basic 10-second recording:**
```bash
python record_video.py --target 5.0 5.0 --duration 10 --output robot_target_5_5.mp4
```

**Advanced recording options:**
```bash
# Record 15 seconds with specific target
python record_video.py --target 7.0 3.0 --duration 15 --output robot_reach_7_3.mp4

# High quality recording (60 FPS)
python record_video.py --target 4.0 4.0 --duration 8 --fps 60 --output high_quality.mp4
```

## Command Options

### main.py Options
- `--target X Y`: Set target position (default: 5.0 5.0)

### record_video.py Options
- `--target X Y`: Set target position (default: 5.0 5.0)
- `--duration SECONDS`: Recording duration (default: 10.0)
- `--output FILENAME`: Output filename (default: robot_simulation.mp4)
- `--fps FPS`: Video FPS (default: 60)

## Simulation Features

### Robot Specifications
- **Number of Links**: 3
- **Link Lengths**: [3.0m, 2.5m, 2.0m]
- **Total Reach**: Maximum 7.5m
- **Joints**: 3 revolute joints

### Control Methods
- **Potential Field**: Attractive force toward target + repulsive force from obstacles
- **Jacobian**: Analytical calculation to convert end-effector force to joint torques
- **Physics Simulation**: Box2D engine

### Visualization
- **Robot**: Gray rectangular links
- **Target**: Red circle with white center point
- **Obstacles**: Gray rectangles
- **Real-time Info**: Target position and frame information

## Example Commands

```bash
# Test with various target positions
python main.py --target 3.0 3.0    # Close target
python main.py --target 7.0 0.0    # Horizontal target  
python main.py --target 5.0 5.0    # Diagonal target
python main.py --target 2.0 -2.0   # Lower target

# Video recording examples
python record_video.py --target 6.0 4.0 --duration 12 --output demo.mp4
```

## Keyboard Shortcuts

- **ESC**: Exit program
- **Close Window**: Terminate simulation

## Troubleshooting

### Robot Not Moving
1. Check force values: If target is too close, force might be too small
2. Check Jacobian calculation: Movement is limited near singularities
3. Set target position within robot's reach (maximum 7.5m)

### Video Recording Error
```bash
# Reinstall OpenCV
pip install --upgrade opencv-python
```

## Developer Information

This project is a robot arm simulation demo using Box2D physics engine.
- **Physics Engine**: Box2D
- **Rendering**: pygame
- **Control**: Potential Field + Jacobian Transpose
- **Video**: OpenCV
