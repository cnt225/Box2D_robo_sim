# Robot Arm Simulation with Box2D

3-link robot arm 2D simulation using Box2D physics engine with pygame visualization. The robot moves to target positions while avoiding obstacles.

## Project Architecture

This project uses a **dual-module architecture** for clean separation of concerns:

### Module 1: Pointcloud Generation (`pointcloud/`)
- **Purpose**: Generate, extract, and manage pointcloud data for environments
- **Key Components**:
  - `create_pointcloud.py` - Generate pointclouds with clustering/obstacle options
  - `pointcloud_extractor.py` - Extract pointclouds from physics environments
  - `pointcloud_loader.py` - Load and reconstruct environments from pointclouds
  - `data/` - Storage for generated pointcloud files (PLY format)

### Module 2: Main Simulation (`main.py`, `record_video.py`)
- **Purpose**: Load pre-generated environments and run robot simulations
- **Key Components**:
  - `main.py` - Real-time simulation with visualization
  - `record_video.py` - Video recording functionality
  - `env.py` - Robot environment setup using geometry configs
  - `robot_config.py` - Pre-defined robot geometry configurations
  - `policy.py` - Control policies (potential field, PD, RMP)
  - `simulation.py` - Core simulation logic
  - `render.py` - Visualization rendering

## Project Structure

```
robot_sim_v1/
├── pointcloud/                    # Module 1: Pointcloud generation & management
│   ├── create_pointcloud.py       # Generate pointclouds with clustering options
│   ├── pointcloud_extractor.py    # Extract pointclouds from environments
│   ├── pointcloud_loader.py       # Load and reconstruct environments
│   └── data/                      # Generated pointcloud files (PLY format)
├── main.py                        # Module 2: Real-time simulation
├── record_video.py                # Video recording functionality
├── env.py                         # Environment setup using geometry configs
├── robot_config.py                # Robot geometry configurations
├── policy.py                      # Control policies
├── simulation.py                  # Core simulation logic
├── render.py                      # Visualization rendering
├── results/                       # Generated video files (auto-created)
├── pyproject.toml                 # Package configuration
└── README.md                      # This file
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

## Workflow Overview

1. **Generate Pointclouds** (Module 1): Use `pointcloud/create_pointcloud.py` to extract pointclouds from environments with various clustering and obstacle options
2. **Run Simulations** (Module 2): Use `main.py` or `record_video.py` to load environments and run robot simulations with simple geometry selection

## Usage

### 1. Generate Pointcloud Environments (Module 1)

First, generate pointcloud data with desired clustering and obstacle configurations:

```bash
# Navigate to pointcloud directory
cd pointcloud

# Generate pointcloud with default settings
python create_pointcloud.py --output_name my_env

# Generate with specific clustering parameters
python create_pointcloud.py --output_name dense_env --clustering_eps 0.2 --min_samples 3

# Generate with different obstacle types
python create_pointcloud.py --output_name circle_env --obstacle_type circle
python create_pointcloud.py --output_name polygon_env --obstacle_type polygon

# Generate with custom resolution and noise
python create_pointcloud.py --output_name precise_env --resolution 0.03 --noise_level 0.005

# See all available options
python create_pointcloud.py --help
```

### 2. Run Simulations (Module 2)

After generating pointclouds, run simulations with simple commands:

#### Basic Simulation

```bash
# List available robot geometries
python main.py --list-geometries

# Run with static environment and geometry #1
python main.py --target 5.0 5.0 --env static --geometry 1 --policy potential_field_pd

# Run with pointcloud environment
python main.py --target 6.0 3.0 --env my_env.ply --geometry 2 --policy rmp

# Use different control policies
python main.py --target 4.0 4.0 --env circle_env.ply --geometry 3 --policy potential_field
```

#### Video Recording

```bash
# Record basic 10-second video
python record_video.py --target 5.0 5.0 --env static --geometry 1 --duration 10 --output demo.mp4

# Record with pointcloud environment
python record_video.py --target 6.0 4.0 --env dense_env.ply --geometry 2 --duration 15 --output pointcloud_demo.mp4

# High quality recording
python record_video.py --target 7.0 3.0 --env polygon_env.ply --geometry 4 --fps 60 --duration 12 --output high_quality.mp4
```

## Command Reference

### Pointcloud Generation (`pointcloud/create_pointcloud.py`)
- `--output_name NAME`: Output filename (without .ply extension)
- `--resolution FLOAT`: Pointcloud resolution (default: 0.05)
- `--noise_level FLOAT`: Noise level for points (default: 0.01)
- `--clustering_eps FLOAT`: DBSCAN clustering epsilon (default: 0.3)
- `--min_samples INT`: DBSCAN minimum samples (default: 5)
- `--obstacle_type TYPE`: Obstacle reconstruction type - "polygon" or "circle" (default: polygon)

### Main Simulation (`main.py`)
- `--target X Y`: Target position (default: 5.0 5.0)
- `--env ENV`: Environment - "static" or PLY filename (default: static)
- `--geometry N`: Robot geometry ID (use --list-geometries to see options)
- `--policy POLICY`: Control policy - "potential_field", "potential_field_pd", or "rmp" (default: potential_field_pd)
- `--list-geometries`: Show available robot geometries

### Video Recording (`record_video.py`)
- `--target X Y`: Target position (default: 5.0 5.0)
- `--env ENV`: Environment - "static" or PLY filename (default: static)
- `--geometry N`: Robot geometry ID
- `--policy POLICY`: Control policy (default: potential_field_pd)
- `--duration SECONDS`: Recording duration (default: 10.0)
- `--output FILENAME`: Output video filename (default: robot_simulation.mp4)
- `--fps FPS`: Video FPS (default: 60)

## Robot Geometry Configurations

The robot supports multiple pre-defined geometry configurations managed in `robot_config.py`:

```bash
# List all available geometries
python main.py --list-geometries
```

Available configurations include:
- **Geometry 1**: Rectangle links [3.0m, 2.5m, 2.0m] with thick profile
- **Geometry 2**: Rectangle links [3.5m, 3.0m, 2.5m] with medium profile  
- **Geometry 3**: Ellipse links [3.0m, 2.5m, 2.0m] with oval shape
- **Geometry 4**: Ellipse links [3.5m, 3.0m, 2.5m] with extended reach
- **Geometry 5**: Rectangle links [2.5m, 2.0m, 1.5m] with compact design
- **Geometry 6**: Ellipse links [4.0m, 3.5m, 3.0m] with maximum reach

Each geometry defines:
- **Link lengths**: Determines robot reach and workspace
- **Link shapes**: Rectangle or ellipse visual appearance
- **Link dimensions**: Width/height for collision detection
- **Total reach**: Maximum distance robot can extend

## Control Policies & Visualization

### Control Methods
- **Potential Field**: Basic attractive force toward target + repulsive force from obstacles
- **Potential Field PD**: Enhanced with PD control for stable convergence (default)
- **RMP (Riemannian Motion Policies)**: Advanced control method using Riemannian metrics
- **Jacobian Transpose**: Converts end-effector forces to joint torques
- **Physics Simulation**: Box2D engine for accurate dynamics

### Visualization Features
- **Robot**: Gray links with configurable shapes (rectangular or elliptical)
- **Target**: Red circle with white center point
- **Obstacles**: Gray rectangles (static) or reconstructed from pointclouds
- **Real-time Info**: Target position, control policy, distance, and geometry information
- **Environment Types**: Static predefined or pointcloud-reconstructed environments

## Complete Workflow Examples

### Example 1: Basic Static Environment

```bash
# List available robot geometries
python main.py --list-geometries

# Run simulation with static environment
python main.py --target 5.0 5.0 --env static --geometry 1 --policy potential_field_pd

# Record video of the same setup
python record_video.py --target 5.0 5.0 --env static --geometry 1 --duration 10 --output static_demo.mp4
```

### Example 2: Pointcloud Environment Workflow

```bash
# Step 1: Generate pointcloud environment
cd pointcloud
python create_pointcloud.py --output_name complex_env --clustering_eps 0.25 --obstacle_type polygon

# Step 2: Use the generated environment
cd ..
python main.py --target 6.0 4.0 --env complex_env.ply --geometry 3 --policy rmp

# Step 3: Record video with the pointcloud environment
python record_video.py --target 6.0 4.0 --env complex_env.ply --geometry 3 --policy rmp --duration 15 --output complex_env_demo.mp4
```

### Example 3: Comparing Different Geometries

```bash
# Generate a test environment
cd pointcloud
python create_pointcloud.py --output_name test_env

# Test different robot geometries on the same environment
cd ..
python main.py --target 7.0 3.0 --env test_env.ply --geometry 1 --policy potential_field_pd  # Compact robot
python main.py --target 7.0 3.0 --env test_env.ply --geometry 6 --policy potential_field_pd  # Extended reach robot

# Record comparison videos
python record_video.py --target 7.0 3.0 --env test_env.ply --geometry 1 --duration 10 --output compact_robot.mp4
python record_video.py --target 7.0 3.0 --env test_env.ply --geometry 6 --duration 10 --output extended_robot.mp4
```

## Advanced Pointcloud Generation

### Available Options for Pointcloud Creation

The `pointcloud/create_pointcloud.py` script supports extensive customization:

```bash
cd pointcloud

# High-resolution environment for detailed simulation
python create_pointcloud.py --output_name high_res_env --resolution 0.02 --noise_level 0.005

# Coarse environment for faster simulation
python create_pointcloud.py --output_name coarse_env --resolution 0.1 --noise_level 0.02

# Dense clustering for complex obstacles
python create_pointcloud.py --output_name dense_obstacles --clustering_eps 0.15 --min_samples 8

# Loose clustering for simpler obstacles  
python create_pointcloud.py --output_name simple_obstacles --clustering_eps 0.4 --min_samples 3

# Different obstacle reconstruction types
python create_pointcloud.py --output_name circular_env --obstacle_type circle
python create_pointcloud.py --output_name polygonal_env --obstacle_type polygon

# See all available options
python create_pointcloud.py --help
```

### Environment Data Management

- **Storage**: All pointcloud files are stored in `pointcloud/data/` as PLY format
- **Metadata**: Clustering and obstacle settings are embedded in PLY file headers
- **Loading**: The main simulation automatically reads metadata when loading PLY files
- **Format**: Only PLY format is supported (legacy formats have been removed)

## Keyboard Shortcuts

- **ESC**: Exit program
- **Close Window**: Terminate simulation

## Troubleshooting

### Robot Not Moving
1. **Target too close**: If target is very close to robot base, forces might be too small
2. **Target out of reach**: Check if target is within robot's maximum reach (varies by geometry)
3. **Singularity issues**: Movement is limited near kinematic singularities  
4. **Obstacle blocking**: Robot may be unable to find path around obstacles

### Video Recording Issues
```bash
# Reinstall OpenCV if video recording fails
uv pip install --upgrade opencv-python
```

### Pointcloud Environment Issues
```bash
# Verify pointcloud file exists
ls pointcloud/data/

# Check file format (only PLY supported)
file pointcloud/data/your_file.ply

# Regenerate pointcloud if loading fails
cd pointcloud
python create_pointcloud.py --output_name new_env
```

### Geometry Selection Issues
```bash
# List available geometries and their properties
python main.py --list-geometries

# Use a valid geometry ID (typically 1-6)
python main.py --target 5.0 5.0 --env static --geometry 1
```

### Environment Loading Issues
```bash
# For static environment, use "static"
python main.py --target 5.0 5.0 --env static --geometry 1

# For pointcloud files, use the PLY filename (with or without .ply extension)
python main.py --target 5.0 5.0 --env my_env.ply --geometry 1
python main.py --target 5.0 5.0 --env my_env --geometry 1
```

## Technical Details

### Architecture Benefits
- **Separation of Concerns**: Pointcloud generation is completely separate from simulation
- **Simplified CLI**: Main simulation only requires 4 core arguments (target, env, geometry, policy)
- **Reusable Environments**: Generate pointclouds once, use in multiple simulations
- **Flexible Geometry**: Easy switching between robot configurations without code changes
- **Batch Processing**: Pointcloud module supports future batch/random generation features

### Dependencies
- **Physics Engine**: Box2D for realistic dynamics simulation
- **Rendering**: pygame for real-time visualization
- **Video Recording**: OpenCV for MP4 video generation
- **Pointcloud Processing**: scikit-learn (DBSCAN clustering), plyfile (PLY format I/O)
- **Control Systems**: NumPy for Jacobian calculations and control algorithms

### File Formats
- **Pointclouds**: PLY format only (legacy formats removed for consistency)
- **Videos**: MP4 format with configurable FPS
- **Metadata**: Clustering and obstacle settings embedded in PLY headers

### Performance Considerations
- **Resolution**: Lower pointcloud resolution (e.g., 0.1) for faster loading
- **Clustering**: Fewer min_samples and larger eps for simpler obstacle reconstruction
- **Geometry**: Simpler geometries (rectangles) render faster than ellipses
- **Video**: Lower FPS (e.g., 30) for smaller file sizes

This project demonstrates a modular approach to robotics simulation with clean separation between environment generation and robot control, making it suitable for research, education, and development workflows.
