# record_video.py - 시뮬레이션 비디오 녹화
import pygame, sys
import numpy as np
import argparse
import cv2
from Box2D.b2 import revoluteJointDef
from env import make_world
from policy import potential_field_policy
from render import draw_world

# 명령행 인자 파싱
def parse_args():
    parser = argparse.ArgumentParser(description='Robot arm simulation with video recording')
    parser.add_argument('--target', nargs=2, type=float, default=[5.0, 5.0],
                        help='Target position (x y), default: 5.0 5.0')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Recording duration in seconds, default: 10.0')
    parser.add_argument('--output', type=str, default='robot_simulation.mp4',
                        help='Output video filename, default: robot_simulation.mp4')
    parser.add_argument('--fps', type=int, default=60,
                        help='Video FPS, default: 60')
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

def main():
    # 1) 명령행 인자 처리
    args = parse_args()
    target = np.array(args.target)
    duration = args.duration
    output_file = args.output
    fps = args.fps
    
    print(f"Target position: {target}")
    print(f"Recording duration: {duration} seconds")
    print(f"Output file: {output_file}")
    print(f"FPS: {fps}")

    # 2) 초기화
    SCREEN_W, SCREEN_H = 800, 600
    TIME_STEP = 1.0 / fps

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption(f"Robot Arm Simulation - Recording - Target: {target}")
    clock = pygame.time.Clock()

    # 3) 월드·로봇·장애물 생성
    world, links, obstacles = make_world()
    joints = world.joints  # 3개 조인트

    # 4) 비디오 작성기 설정
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (SCREEN_W, SCREEN_H))

    # 5) 녹화 변수
    total_frames = int(duration * fps)
    current_frame = 0

    print(f"Starting recording... Total frames: {total_frames}")

    # 6) 메인 루프
    running = True
    while running and current_frame < total_frames:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

        # policy → planar force
        F = potential_field_policy(links, target, obstacles)

        # Jacobian J2 (2×3) 해석적 계산
        L = [3.0, 2.5, 2.0]  # 링크 길이
        
        # 각 조인트의 현재 각도 (누적)
        q1 = joints[0].angle if len(joints) > 0 else 0
        q2 = q1 + joints[1].angle if len(joints) > 1 else q1
        q3 = q2 + joints[2].angle if len(joints) > 2 else q2
        
        # Jacobian 계산
        J2 = np.zeros((2, 3))
        
        J2[0, 0] = -L[0]*np.sin(q1) - L[1]*np.sin(q2) - L[2]*np.sin(q3)
        J2[1, 0] = L[0]*np.cos(q1) + L[1]*np.cos(q2) + L[2]*np.cos(q3)
        
        J2[0, 1] = -L[1]*np.sin(q2) - L[2]*np.sin(q3)
        J2[1, 1] = L[1]*np.cos(q2) + L[2]*np.cos(q3)
        
        J2[0, 2] = -L[2]*np.sin(q3)
        J2[1, 2] = L[2]*np.cos(q3)

        # torque via J^T F
        tau = J2.T.dot(F)

        # torque 적용
        for j, t in zip(joints, tau):
            j.bodyB.ApplyTorque(t, wake=True)

        # 물리 한 스텝
        world.Step(TIME_STEP, 10, 10)

        # 렌더링
        screen.fill((30, 30, 30))
        draw_world(screen, world, SCREEN_W, SCREEN_H)
        
        # target 위치 시각화
        draw_target(screen, target)
        
        # 정보 텍스트 표시
        font = pygame.font.Font(None, 36)
        info_text = f"Target: ({target[0]:.1f}, {target[1]:.1f}) | Frame: {current_frame}/{total_frames}"
        text_surface = font.render(info_text, True, (255, 255, 255))
        screen.blit(text_surface, (10, 10))
        
        # 진행률 표시
        progress = current_frame / total_frames
        progress_text = f"Recording: {progress*100:.1f}%"
        progress_surface = font.render(progress_text, True, (255, 255, 0))
        screen.blit(progress_surface, (10, 50))
        
        # 화면을 numpy 배열로 변환
        pygame_image = pygame.surfarray.array3d(screen)
        # pygame은 (width, height, 3), opencv는 (height, width, 3) 형식
        opencv_image = np.transpose(pygame_image, (1, 0, 2))
        # RGB를 BGR로 변환 (opencv 형식)
        opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_RGB2BGR)
        
        # 비디오 프레임 작성
        video_writer.write(opencv_image)
        
        pygame.display.flip()
        clock.tick(fps)
        
        current_frame += 1
        
        # 진행률 출력 (10% 단위)
        if current_frame % (total_frames // 10) == 0:
            print(f"Recording progress: {current_frame}/{total_frames} frames ({progress*100:.0f}%)")

    # 7) 정리
    video_writer.release()
    pygame.quit()
    
    print(f"Recording completed! Video saved as: {output_file}")
    print(f"Total frames recorded: {current_frame}")

if __name__ == "__main__":
    main()
