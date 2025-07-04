# main.py
import pygame, sys
import numpy as np
import argparse
from Box2D.b2 import revoluteJointDef
from env import make_world
from policy import potential_field_policy, rmp_policy
from policy import rmp_policy, cbf_qp_policy    
from render import draw_world

# 명령행 인자 파싱
def parse_args():
    parser = argparse.ArgumentParser(description='Robot arm simulation')
    parser.add_argument('--target', nargs=2, type=float, default=[5.0, 5.0],
                        help='Target position (x y), default: 5.0 5.0')
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
world, links, obstacles = make_world()
joints = world.joints  # 3개 조인트

# 3) 루프
while True:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            pygame.quit(); sys.exit()

    # 4) policy → planar force
    # F = potential_field_policy(links, target, obstacles)
    F = rmp_policy(links, target, obstacles)

    # 디버깅: force 값 출력
    end_pos = np.array(links[-1].worldCenter)
    print(f"End-effector: {end_pos}, Target: {target}, Force: {F}")

    # 5) Jacobian J2 (2×3) 해석적 계산
    # 각 조인트의 현재 각도와 링크 길이를 이용해 해석적으로 계산
    L = [3.0, 2.5, 2.0]  # 링크 길이
    
    # 각 조인트의 현재 각도 (누적)
    q1 = joints[0].angle if len(joints) > 0 else 0
    q2 = q1 + joints[1].angle if len(joints) > 1 else q1
    q3 = q2 + joints[2].angle if len(joints) > 2 else q2
    
    # End-effector 위치 = base + L1*[cos(q1), sin(q1)] + L2*[cos(q2), sin(q2)] + L3*[cos(q3), sin(q3)]
    # Jacobian 계산: dP/dq_i
    J2 = np.zeros((2, 3))
    
    # dP/dq1 = -L1*sin(q1) - L2*sin(q2) - L3*sin(q3), L1*cos(q1) + L2*cos(q2) + L3*cos(q3)
    J2[0, 0] = -L[0]*np.sin(q1) - L[1]*np.sin(q2) - L[2]*np.sin(q3)
    J2[1, 0] = L[0]*np.cos(q1) + L[1]*np.cos(q2) + L[2]*np.cos(q3)
    
    # dP/dq2 = -L2*sin(q2) - L3*sin(q3), L2*cos(q2) + L3*cos(q3)
    J2[0, 1] = -L[1]*np.sin(q2) - L[2]*np.sin(q3)
    J2[1, 1] = L[1]*np.cos(q2) + L[2]*np.cos(q3)
    
    # dP/dq3 = -L3*sin(q3), L3*cos(q3)
    J2[0, 2] = -L[2]*np.sin(q3)
    J2[1, 2] = L[2]*np.cos(q3)
    
    print(f"Joint angles: [{q1:.3f}, {q2:.3f}, {q3:.3f}]")
    print(f"Jacobian:\n{J2}")

    # 6) torque via J^T F
    tau = J2.T.dot(F)
    
    # 디버깅: torque 값 출력
    print(f"Torques: {tau}")

    # 7) torque 적용
    for j, t in zip(joints, tau):
        j.bodyB.ApplyTorque(t, wake=True)

    # 8) 물리 한 스텝
    world.Step(TIME_STEP, 10, 10)

    # 9) 렌더링
    draw_world(screen, world, SCREEN_W, SCREEN_H)
    
    # target 위치 시각화
    draw_target(screen, target)
    
    # 정보 텍스트 표시
    font = pygame.font.Font(None, 36)
    info_text = f"Target: ({target[0]:.1f}, {target[1]:.1f})"
    text_surface = font.render(info_text, True, (255, 255, 255))
    screen.blit(text_surface, (10, 10))
    
    pygame.display.flip()
    clock.tick(FPS)