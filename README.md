# Robot Arm Simulation with Box2D

3-link robot arm 2D simulation using Box2D physics engine with pygame visualization. The robot moves to target positions while avoiding obstacles.

## Project Architecture

이 프로젝트는 **다중 모듈 아키텍처**로 기능을 명확히 분리하여 구성되어 있습니다:

### Module 1: Pointcloud Generation (`pointcloud/`)
- **Purpose**: Generate, extract, and manage pointcloud data for environments
- **Key Components**:
  - `create_pointcloud.py` - Generate pointclouds with clustering/obstacle options
  - `pointcloud_loader.py` - Load and reconstruct environments from pointclouds
  - `utils/visualize_pointcloud.py` - Visualize pointcloud data with robot base position
  - `utils/quick_visualize.py` - 🆕 **Quick environment visualization without regenerating all images**
  - `random_environment_generator.py` - Generate diverse environments
  - `concave_shape_generator/` - 🆕 **Complete concave obstacle generation system**
  - `data/` - **Organized folder structure**: `data/env_name/` with PLY, metadata, and images

### Module 2: Pose Generation (`pose/`)
- **Purpose**: 🆕 **Generate random robot poses and collision detection**
- **Key Components**:
  - `random_pose_generator.py` - Generate random valid robot poses
  - `collision_detector.py` - Detect collisions between robot and environment
  - `pose_pipeline.py` - Complete automated pose generation workflow

### Module 3: Main Simulation (`main.py`, `record_video.py`)
- **Purpose**: Load pre-generated environments and run robot simulations
- **Key Components**:
  - `main.py` - Real-time simulation with visualization
  - `record_video.py` - Video recording functionality
  - `env.py` - Robot environment setup using geometry configs
  - `policy.py` - Control policies (potential field, PD, RMP)
  - `simulation.py` - Core simulation logic
  - `render.py` - Visualization rendering

## ✨ Latest Major Features (2025-01-2025)

### 🎯 **Quick Environment Visualization System**
빠른 환경 조회 및 시각화를 위한 새로운 도구:
```bash
# 특정 환경 빠른 시각화 (예: #65번 환경)
python pointcloud/utils/quick_visualize.py circle_envs_10k 65

# 환경 정보와 함께 시각화
python pointcloud/utils/quick_visualize.py random_hard_01 --show-info
```

### 🔷 **Concave Obstacle Generation System**
복잡한 오목 형태 장애물 생성 및 Box2D 통합:
- **Concave Shape Generator**: L-shape, U-shape 등 복잡한 형태 생성
- **Box2D Integration**: 자동 Convex Decomposition으로 물리 엔진 호환성
- **Multiple Methods**: Triangulation, Convex Hull, Multiple Fixtures 지원
- **Complete Pipeline**: SVG → JSON → Box2D Bodies 완전 자동화

**파일 구조:**
```
pointcloud/concave_shape_generator/
├── concave_shape_generator.py     # 오목 형태 생성기
├── concave_box2d_integration.py   # Box2D 통합 및 변환
├── concave_environment_generator.py # 환경 배치 시스템
└── shapes/                        # 생성된 SVG/JSON 라이브러리
```

### 🤖 **Robot Pose Generation System**
로봇 포즈 생성 및 충돌 검사 시스템:
- **Random Pose Generator**: 6가지 로봇 구성별 랜덤 포즈 생성
- **Collision Detection**: PLY 환경과 로봇 링크 간 정밀 충돌 검사
- **Complete Pipeline**: 환경 로드 → 포즈 생성 → 충돌 검사 → 결과 저장

**성능:**
- **392+ poses/sec** 처리 속도
- **Rectangle/Ellipse** 링크 형태 모두 지원
- **JSON 형식** 결과 저장 (통계 포함)

**사용법:**
```bash
# 충돌 없는 포즈 20개 생성
python pose/pose_pipeline.py data/pointcloud/circles_only/circles_only.ply 0 --num_poses 20 --output my_poses.json

# 분석 포함 생성
python pose/pose_pipeline.py data/pointcloud/random_hard_01/random_hard_01.ply 2 --analyze --output hard_poses.json
```

### 📁 **Enhanced File Organization**
- **대용량 데이터 .gitignore 처리**: 557MB 포인트클라우드 데이터 제외
- **구조적 정리**: 기능별 모듈화 (pointcloud/, pose/, 메인 시뮬레이션)
- **자동 빌드 제외**: PLY, JSON, 임시 파일들 git 추적 제외

## Project Structure

```
robot_sim_v1/
├── pointcloud/                           # Module 1: Pointcloud & Environment
│   ├── create_pointcloud.py              # Generate pointclouds
│   ├── pointcloud_loader.py              # Load environments
│   ├── random_environment_generator.py   # Generate diverse environments
│   ├── utils/                            # 🆕 Utility functions
│   │   ├── visualize_pointcloud.py       # Visualize environments
│   │   ├── quick_visualize.py            # Quick environment viewer
│   │   └── svg_to_ply_converter.py       # SVG conversion utility
│   ├── concave_shape_generator/          # 🆕 Concave obstacle system
│   │   ├── concave_shape_generator.py    # Shape generation
│   │   ├── concave_box2d_integration.py  # Box2D integration
│   │   ├── concave_environment_generator.py # Environment placement
│   │   └── shapes/                       # Shape library (SVG/JSON)
│   └── data/                             # Environment data folders
│       ├── circles_only/                 # Circle-only environments
│       ├── rectangles_only/              # Rectangle environments
│       ├── polygons_only/                # Polygon environments
│       ├── random_*/                     # Mixed environments
│       └── circle_envs_10k/              # 🎯 Large-scale dataset
├── pose/                                 # Module 2: Pose generation
│   ├── random_pose_generator.py          # Core pose generation
│   ├── batch_pose_generator.py           # Batch pose generation
│   ├── pose_visualizer.py                # Pose visualization
│   ├── collision_detector.py             # Collision detection
│   └── pose_pipeline.py                  # Complete workflow
├── data/                                 # 🆕 Unified data directory
│   ├── pointcloud/                       # Environment pointcloud data
│   ├── pose/                             # Generated pose data
│   └── results/                          # Results directory
│       ├── poses/                        # Pose visualization images
│       └── simulation_videos/            # Simulation recordings
├── main.py                               # Module 3: Real-time simulation
├── record_video.py                       # Video recording
├── env.py                                # Environment setup
├── policy.py                             # Control policies
├── simulation.py                         # Core simulation logic
├── render.py                             # Visualization rendering
├── config.yaml                           # Central configuration
└── README.md                             # This file
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
# Install all required packages
uv pip install -e .

# For development (includes Jupyter support)
uv pip install -e ".[dev]"
```

## Workflow Overview

1. **Generate Environments**: Use `pointcloud/create_pointcloud.py` for basic environments
2. **Create Concave Obstacles**: Use `concave_shape_generator/` for complex shapes
3. **Quick Visualization**: Use `quick_visualize.py` for fast environment inspection
4. **Generate Robot Poses**: Use `pose/pose_pipeline.py` for collision-free poses
5. **Run Simulations**: Use `main.py` or `record_video.py` for robot control

## Usage

### 1. Quick Environment Visualization 🆕

빠른 환경 조회 및 시각화:

```bash
# 특정 환경 번호로 빠른 조회
python pointcloud/utils/quick_visualize.py circle_envs_10k 65

# 환경 정보 표시
python pointcloud/utils/quick_visualize.py random_hard_01 --show-info

# 이미지 저장
python pointcloud/utils/quick_visualize.py polygons_only --save-image
```

### 2. Concave Obstacle Generation 🆕

복잡한 오목 형태 장애물 생성:

```bash
# 기본 L-shape 장애물 생성
python pointcloud/concave_shape_generator/concave_shape_generator.py

# Box2D 통합 테스트
python pointcloud/concave_shape_generator/concave_box2d_integration.py

# 완전한 환경에 배치
python pointcloud/concave_shape_generator/concave_environment_generator.py
```

### 3. Robot Pose Generation 🆕

로봇 포즈 생성 및 충돌 검사:

```bash
# 기본 포즈 생성 (20개)
python pose/pose_pipeline.py data/pointcloud/circles_only/circles_only.ply 0 --num_poses 20

# JSON 파일로 저장 + 분석
python pose/pose_pipeline.py data/pointcloud/random_hard_01/random_hard_01.ply 2 --num_poses 50 --output hard_poses.json --analyze

# 고급 옵션 (안전 여유거리, 최대 시도 횟수)
python pose/pose_pipeline.py data/pointcloud/polygons_only/polygons_only.ply 1 --num_poses 100 --safety_margin 0.1 --max_attempts 2000
```

### 4. Generate Pointcloud Environments

다양한 환경 타입 생성:

```bash
# Navigate to pointcloud directory
cd pointcloud

# Generate basic environment types
python create_pointcloud.py --env-type random --obstacle-types circle --output-name circles_only --seed 111
python create_pointcloud.py --env-type random --obstacle-types rectangle --output-name rectangles_only --seed 222

# Generate mixed difficulty environments
python create_pointcloud.py --env-type random --difficulty easy --output-name random_easy_01 --seed 42
python create_pointcloud.py --env-type random --difficulty medium --output-name random_medium_01 --seed 456
python create_pointcloud.py --env-type random --difficulty hard --output-name random_hard_01 --seed 789
```

### 5. Run Simulations

기본 시뮬레이션 실행:

```bash
# List available robot geometries
python main.py --list-geometries

# Run with specific environment
python main.py --target 6.0 3.0 --env circles_only --geometry 2 --policy rmp
python main.py --target 8.0 6.0 --env random_medium_01 --geometry 3 --policy potential_field
```

#### Video Recording

```bash
# Record with organized environment
python record_video.py --target 6.0 4.0 --env random_medium_01 --geometry 2 --duration 15 --output medium_demo.mp4

# High quality recording
python record_video.py --target 7.0 3.0 --env polygons_only --geometry 4 --fps 60 --duration 12 --output polygon_demo.mp4
```

## 🔧 Command Reference

### Quick Visualization (`pointcloud/utils/quick_visualize.py`) 🆕
- `env_name`: Environment name or folder
- `env_number`: Specific environment number (for large datasets)
- `--show-info`: Display environment metadata
- `--save-image`: Save visualization as image
- `--no-show`: Don't display plot

### Pose Generation (`pose/pose_pipeline.py`) 🆕
- `ply_file`: PLY environment file path
- `robot_id`: Robot ID (0-5)
- `--num_poses`: Number of collision-free poses (default: 100)
- `--output`: Output JSON filename
- `--analyze`: Perform pose distribution analysis
- `--safety_margin`: Safety margin for collision detection (default: 0.05)
- `--max_attempts`: Maximum generation attempts (default: 1000)
- `--seed`: Random seed (default: 42)

### Pointcloud Generation (`pointcloud/create_pointcloud.py`)
**Basic Options:**
- `--env-type TYPE`: Environment type - "static" or "random"
- `--output-name NAME`: Output filename
- `--difficulty LEVEL`: For random environments - "easy", "medium", "hard"
- `--obstacle-types TYPES`: Space-separated list - "rectangle", "circle", "polygon", "curve"
- `--seed INT`: Random seed for reproducible environments

**Advanced Options:**
- `--resolution FLOAT`: Pointcloud resolution in meters (default: 0.05)
- `--noise-level FLOAT`: Sensor noise level (default: 0.01)
- `--clustering-eps FLOAT`: DBSCAN clustering epsilon (default: 0.3)
- `--min-samples INT`: DBSCAN minimum samples (default: 5)

### Main Simulation (`main.py`)
- `--target X Y`: Target position (default: 5.0 5.0)
- `--env ENV`: Environment name or PLY filename (default: static)
- `--geometry N`: Robot geometry ID (use --list-geometries to see options)
- `--policy POLICY`: Control policy - "potential_field", "potential_field_pd", or "rmp"
- `--list-geometries`: Show available robot geometries

### Video Recording (`record_video.py`)
- `--target X Y`: Target position (default: 5.0 5.0)
- `--env ENV`: Environment name or PLY filename (default: static)
- `--geometry N`: Robot geometry ID
- `--policy POLICY`: Control policy (default: potential_field_pd)
- `--duration SECONDS`: Recording duration (default: 10.0)
- `--output FILENAME`: Output video filename (default: robot_simulation.mp4)
- `--fps FPS`: Video FPS (default: 60)

## Robot Geometry Configurations

6가지 사전 정의된 로봇 구성을 지원합니다:

```bash
# List all available geometries
python main.py --list-geometries
```

- **Robot 0**: Standard Rectangle Robot (3.0/2.5/2.0 lengths)
- **Robot 1**: Compact Robot (2.0/1.5/1.0 lengths) 
- **Robot 2**: Extended Robot (4.0/3.0/2.5 lengths)
- **Robot 3**: Standard Ellipse Robot (elliptical links)
- **Robot 4**: Narrow Ellipse Robot (thin elliptical links)
- **Robot 5**: Wide Ellipse Robot (wide elliptical links)

## Data Formats

### Pose Data Format (JSON)
```json
{
  "robot_id": 0,
  "num_poses": 20,
  "poses": [
    [1.2449, -0.5018, -1.0823],  // Joint angles in radians
    [0.9301, 0.3428, -1.0331],   // [joint1, joint2, joint3]
    // ... more poses
  ],
  "statistics": {
    "collision_rate": 0.32,
    "success_rate": 0.68,
    "poses_per_second": 392.4,
    "environment_file": "circles_only.ply"
  },
  "format": "joint_angles_radians"
}
```

### Environment Metadata Format (JSON)
```json
{
  "environment_name": "circles_only",
  "difficulty": "medium", 
  "num_obstacles": 16,
  "num_points": 7427,
  "generation_parameters": {
    "seed": 111,
    "resolution": 0.03,
    "obstacle_types": ["circle"]
  }
}
```

## Performance Statistics

- **Pose Generation**: 392+ poses/sec
- **Environment Loading**: <0.1s for typical environments
- **Collision Detection**: Real-time for 10K+ point environments
- **Large Dataset**: 21% of 10K circle environments (4,238 files, 205MB)

## Development Status

### ✅ Completed Features
- Dual-module architecture (pointcloud + simulation)
- 8 pre-generated environment types
- Advanced shape detection and reconstruction
- Quick environment visualization system
- Complete concave obstacle generation pipeline
- Robot pose generation with collision detection
- Video recording and real-time simulation
- Multiple robot geometries and control policies

### 🚧 Current Focus
- Large-scale pose dataset generation
- Trajectory planning integration
- Path validation between poses
- Advanced collision detection optimization

---

*For detailed development history and technical improvements, see the project's Git history and TODO.md file.*
