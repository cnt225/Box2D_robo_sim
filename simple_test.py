# simple_test.py - 간단한 torque 테스트
import pygame, sys
import numpy as np
from env import make_world
from render import draw_world

# 초기화
SCREEN_W, SCREEN_H = 800, 600
FPS = 60
TIME_STEP = 1.0 / FPS

pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Simple Torque Test")
clock = pygame.time.Clock()

# 월드 생성
world, links, obstacles = make_world()
joints = world.joints

print(f"Number of joints: {len(joints)}")
print(f"Initial end-effector position: {links[-1].worldCenter}")

# 메인 루프
frame_count = 0
while True:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            pygame.quit(); sys.exit()

    # 간단한 고정 torque 적용
    torques = [1.0, 0.5, -0.3]  # 각 조인트에 다른 torque
    
    for j, t in zip(joints, torques):
        j.bodyB.ApplyTorque(t, wake=True)
    
    # 매 60프레임마다 위치 출력
    if frame_count % 60 == 0:
        print(f"Frame {frame_count}: End-effector at {links[-1].worldCenter}")
    
    frame_count += 1

    # 물리 시뮬레이션
    world.Step(TIME_STEP, 10, 10)

    # 렌더링
    screen.fill((30, 30, 30))
    draw_world(screen, world, SCREEN_W, SCREEN_H)
    
    # 정보 표시
    font = pygame.font.Font(None, 36)
    info = f"Frame: {frame_count}, End-pos: {links[-1].worldCenter}"
    text = font.render(info[:60], True, (255, 255, 255))
    screen.blit(text, (10, 10))
    
    pygame.display.flip()
    clock.tick(FPS)
