# main.py - 모듈 2: 시뮬레이션 실행
import pygame, sys
import argparse
import numpy as np
from env import make_world, list_available_pointclouds
from simulation import RobotSimulation
from render import draw_world

# 명령행 인자 파싱
def parse_args():
    parser = argparse.ArgumentParser(description='Robot arm simulation')
    parser.add_argument('--target', nargs=2, type=float, default=[5.0, 5.0],
                        help='Target position (x y), default: 5.0 5.0')
    parser.add_argument('--env', type=str, default=None,
                        help='Pointcloud environment file (.ply). If not specified, uses static environment')
    parser.add_argument('--geometry', type=int, default=0,
                        help='Robot geometry ID (0-5), default: 0. Use --list-geometries to see available options')
    parser.add_argument('--policy', choices=['potential_field', 'potential_field_pd', 'rmp'], 
                        default='potential_field_pd',
                        help='Control policy (default: potential_field_pd)')
    parser.add_argument('--list-geometries', action='store_true',
                        help='List available robot geometries and exit')
    
    return parser.parse_args()

# target 위치 시각화 함수
def draw_target(screen, target, PPM=50.0, ORIGIN=(100, 500)):
    # 월드 좌표를 화면 좌표로 변환
    x, y = target
    sx = ORIGIN[0] + x * PPM
    sy = ORIGIN[1] - y * PPM
    
    # 빨간 원으로 target 그리기
    pygame.draw.circle(screen, (255, 0, 0), (int(sx), int(sy)), 8)
    # 중심점 표시
    pygame.draw.circle(screen, (255, 255, 255), (int(sx), int(sy)), 2)

# 1) 명령행 인자 처리
args = parse_args()

# Handle --list-geometries flag
if args.list_geometries:
    from config_loader import get_config
    config = get_config()
    print("Available Robot Geometries:")
    for geo_id, geo_config in config.get_robot_geometries().items():
        shape = geo_config['link_shape']
        name = geo_config['name']
        desc = geo_config['description']
        lengths = geo_config['link_lengths']
        max_reach = geo_config.get('max_reach', 'N/A')
        print(f"  {geo_id}: {name} ({shape}) - {desc}")
        print(f"      Lengths: {lengths} | Max Reach: {max_reach}")
        print()
    sys.exit(0)

# Validate pointcloud file if specified
if args.env and args.env != 'static':
    # Check if pointcloud file exists
    import os
    if not args.env.endswith('.ply'):
        args.env += '.ply'
    
    pointcloud_path = os.path.join("pointcloud", "data", args.env)
    if not os.path.exists(pointcloud_path):
        print(f"Error: Pointcloud file not found: {pointcloud_path}")
        print("Available pointcloud files:")
        available = list_available_pointclouds()
        if available:
            for file in available:
                print(f"  - {file}.ply")
        else:
            print("  (No pointcloud files found)")
        print("\nCreate pointclouds with: python create_pointcloud.py <n>")
        sys.exit(1)

# Determine environment type
env_type = 'static' if args.env == 'static' else 'pointcloud'

# Parse target position
target = np.array(args.target)

print(f"Environment: {env_type}")
print(f"Target position: {target}")
if env_type == 'pointcloud':
    print(f"Pointcloud file: {args.env}")
print(f"Robot geometry: {args.geometry}")

# 2) 초기화
SCREEN_W, SCREEN_H = 800, 600
FPS = 60
TIME_STEP = 1.0 / FPS

pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption(f"Robot Arm Simulation - Target: {target}")
clock  = pygame.time.Clock()

# 3) 월드·로봇·장애물 생성
world, links, obstacles = make_world(
    geometry_id=args.geometry,
    env_file=args.env
)

# 4) 시뮬레이션 객체 생성
simulation = RobotSimulation(world, links, obstacles, target, args.policy)

# 5) 메인 루프
while True:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            pygame.quit(); sys.exit()

    # 시뮬레이션 한 스텝 실행
    step_info = simulation.step(debug=True)

    # 렌더링
    draw_world(screen, world, SCREEN_W, SCREEN_H)
    
    # target 위치 시각화
    draw_target(screen, target)
    
    # 정보 텍스트 표시
    font = pygame.font.Font(None, 36)
    info_text = f"Target: ({target[0]:.1f}, {target[1]:.1f}) | Policy: {args.policy} | Env: {args.env}"
    text_surface = font.render(info_text, True, (255, 255, 255))
    screen.blit(text_surface, (10, 10))
    
    # 거리 정보 표시
    distance = simulation.get_distance_to_target()
    dist_text = f"Distance to target: {distance:.3f}m"
    dist_surface = font.render(dist_text, True, (255, 255, 255))
    screen.blit(dist_surface, (10, 50))
    
    pygame.display.flip()
    clock.tick(FPS)