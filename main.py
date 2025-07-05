# main.py
import pygame, sys
import numpy as np
import argparse
from env import make_world
from simulation import RobotSimulation
from render import draw_world

# config.yaml 읽기 (간단하게)
try:
    import yaml
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    link_shape = config['robot']['link_shape']
except:
    link_shape = "rectangle"  # 기본값

# 명령행 인자 파싱
def parse_args():
    parser = argparse.ArgumentParser(description='Robot arm simulation')
    parser.add_argument('--target', nargs=2, type=float, default=[5.0, 5.0],
                        help='Target position (x y), default: 5.0 5.0')
    parser.add_argument('--link-shape', choices=['rectangle', 'ellipse'], default=link_shape,
                        help=f'Link shape (default: {link_shape})')
    parser.add_argument('--policy', choices=['potential_field', 'potential_field_pd', 'rmp'], 
                        default='potential_field_pd',
                        help='Control policy (default: potential_field_pd)')
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
target = np.array(args.target)
print(f"Target position: {target}")

# 2) 초기화
SCREEN_W, SCREEN_H = 800, 600
FPS = 60
TIME_STEP = 1.0 / FPS

pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption(f"Robot Arm Simulation - Target: {target}")
clock  = pygame.time.Clock()

# 3) 월드·로봇·장애물 생성
world, links, obstacles = make_world(args.link_shape)

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
    info_text = f"Target: ({target[0]:.1f}, {target[1]:.1f}) | Policy: {args.policy}"
    text_surface = font.render(info_text, True, (255, 255, 255))
    screen.blit(text_surface, (10, 10))
    
    # 거리 정보 표시
    distance = simulation.get_distance_to_target()
    dist_text = f"Distance to target: {distance:.3f}m"
    dist_surface = font.render(dist_text, True, (255, 255, 255))
    screen.blit(dist_surface, (10, 50))
    
    pygame.display.flip()
    clock.tick(FPS)