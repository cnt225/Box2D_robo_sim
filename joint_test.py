# joint_test.py - 각 조인트가 서로 다른 속도로 등속회전하는 테스트
import pygame, sys
import numpy as np
import math
from env import make_world
from render import draw_world

# 1) 초기화
SCREEN_W, SCREEN_H = 800, 600
FPS = 60
TIME_STEP = 1.0 / FPS

pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Joint Rotation Test")
clock = pygame.time.Clock()

# 2) 월드·로봇·장애물 생성
world, links, obstacles = make_world()
joints = world.joints  # 3개 조인트

# 3) 각 조인트의 회전 속도 설정 (rad/s)
joint_speeds = [1.0, 0.5, -0.8]  # 첫 번째: 1 rad/s, 두 번째: 0.5 rad/s, 세 번째: -0.8 rad/s

print(f"조인트 개수: {len(joints)}")
print(f"설정된 속도: {joint_speeds}")

# 4) 시간 카운터
time_elapsed = 0.0

# 5) 메인 루프
while True:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()

    # 시간 업데이트
    time_elapsed += TIME_STEP
    
    # 각 조인트에 등속 회전 적용
    for i, (joint, speed) in enumerate(zip(joints, joint_speeds)):
        # 조인트의 현재 각속도를 설정
        joint.bodyB.angularVelocity = speed
        
        # 또는 직접 토크를 적용하여 목표 속도 유지
        current_velocity = joint.bodyB.angularVelocity
        target_velocity = speed
        velocity_error = target_velocity - current_velocity
        
        # 간단한 PD 제어
        kp = 10.0  # 비례 게인
        kd = 2.0   # 미분 게인
        
        torque = kp * velocity_error - kd * current_velocity
        joint.bodyB.ApplyTorque(torque, wake=True)

    # 물리 시뮬레이션 한 스텝
    world.Step(TIME_STEP, 10, 10)

    # 화면 렌더링
    screen.fill((30, 30, 30))
    draw_world(screen, world, SCREEN_W, SCREEN_H)
    
    # 정보 텍스트 출력
    font = pygame.font.Font(None, 36)
    info_text = f"Time: {time_elapsed:.1f}s | ESC to quit"
    text_surface = font.render(info_text, True, (255, 255, 255))
    screen.blit(text_surface, (10, 10))
    
    # 각 조인트 정보
    for i, (joint, speed) in enumerate(zip(joints, joint_speeds)):
        angle = joint.bodyB.angle
        velocity = joint.bodyB.angularVelocity
        info = f"Joint {i+1}: {math.degrees(angle):.1f}° ({velocity:.2f} rad/s, target: {speed:.2f})"
        text = font.render(info, True, (255, 255, 0))
        screen.blit(text, (10, 50 + i * 30))
    
    pygame.display.flip()
    clock.tick(FPS)
