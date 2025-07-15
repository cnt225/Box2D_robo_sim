#!/usr/bin/env python3
"""
Collision Detector
PLY 포인트클라우드 환경과 주어진 로봇 포즈 간의 충돌을 검사하는 모듈

충돌 검사 방법:
1. PLY 파일에서 장애물 포인트들 로드
2. 로봇 링크를 기하학적 형태 (rectangle/ellipse)로 모델링
3. 포인트-형태 충돌 검사 수행
"""

import numpy as np
import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass

try:
    from .random_pose_generator import RandomPoseGenerator, RobotGeometry
except ImportError:
    from random_pose_generator import RandomPoseGenerator, RobotGeometry


@dataclass
class CollisionResult:
    """충돌 검사 결과"""
    is_collision: bool
    colliding_links: List[int]  # 충돌한 링크 번호들
    collision_points: List[Tuple[float, float]]  # 충돌 지점들
    num_collision_points: int


class CollisionDetector:
    """충돌 검사기"""
    
    def __init__(self, pose_generator: Optional[RandomPoseGenerator] = None):
        """
        Args:
            pose_generator: 포즈 생성기 (로봇 geometry 정보용)
                           None이면 새로 생성
        """
        if pose_generator is None:
            self.pose_generator = RandomPoseGenerator()
        else:
            self.pose_generator = pose_generator
        
        self.environment_points = None
        self.environment_bounds = None
    
    def load_environment(self, ply_file: str) -> bool:
        """
        PLY 파일에서 환경 포인트클라우드 로드
        
        Args:
            ply_file: PLY 파일 경로
            
        Returns:
            로드 성공 여부
        """
        try:
            points = self._read_ply_file(ply_file)
            
            if len(points) == 0:
                print(f"Warning: No points found in {ply_file}")
                return False
            
            self.environment_points = np.array(points)
            
            # 환경 경계 계산
            min_x, min_y = np.min(self.environment_points, axis=0)
            max_x, max_y = np.max(self.environment_points, axis=0)
            self.environment_bounds = (min_x, max_x, min_y, max_y)
            
            print(f"Loaded environment: {len(points)} points")
            print(f"Environment bounds: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]")
            
            return True
            
        except Exception as e:
            print(f"Error loading PLY file {ply_file}: {e}")
            return False
    
    def _read_ply_file(self, ply_file: str) -> List[Tuple[float, float]]:
        """PLY 파일에서 2D 포인트 데이터 읽기"""
        points = []
        
        with open(ply_file, 'r') as f:
            # 헤더 스킵
            line = f.readline()
            while not line.startswith('end_header'):
                line = f.readline()
            
            # 포인트 데이터 읽기
            for line in f:
                if line.strip():
                    coords = line.strip().split()
                    if len(coords) >= 3:
                        x, y = float(coords[0]), float(coords[1])
                        points.append((x, y))
        
        return points
    
    def check_collision(self, pose: List[float], robot_id: int, 
                       safety_margin: float = 0.05) -> CollisionResult:
        """
        포즈와 환경 간 충돌 검사
        
        Args:
            pose: [theta1, theta2, theta3] 관절 각도 (라디안)
            robot_id: 로봇 ID
            safety_margin: 안전 여유 거리
            
        Returns:
            CollisionResult 객체
        """
        
        if self.environment_points is None:
            print("Error: Environment not loaded. Call load_environment() first.")
            return CollisionResult(False, [], [], 0)
        
        geometry = self.pose_generator.get_robot_geometry(robot_id)
        if geometry is None:
            print(f"Error: Robot ID {robot_id} not found")
            return CollisionResult(False, [], [], 0)
        
        # 모든 링크 위치 계산
        link_positions = self.pose_generator.get_all_link_positions(pose, geometry)
        
        colliding_links = []
        collision_points = []
        
        # 각 링크에 대해 충돌 검사
        for link_idx in range(len(link_positions) - 1):  # 마지막은 end-effector이므로 제외
            link_collisions = self._check_link_collision(
                link_positions[link_idx], 
                link_positions[link_idx + 1],
                geometry.link_widths[link_idx],
                geometry.link_shape,
                safety_margin
            )
            
            if link_collisions:
                colliding_links.append(link_idx)
                collision_points.extend(link_collisions)
        
        is_collision = len(colliding_links) > 0
        
        return CollisionResult(
            is_collision=is_collision,
            colliding_links=colliding_links,
            collision_points=collision_points,
            num_collision_points=len(collision_points)
        )
    
    def _check_link_collision(self, start_pos: Tuple[float, float], 
                             end_pos: Tuple[float, float],
                             link_width: float, link_shape: str,
                             safety_margin: float) -> List[Tuple[float, float]]:
        """
        단일 링크와 환경 간 충돌 검사
        
        Args:
            start_pos: 링크 시작점
            end_pos: 링크 끝점
            link_width: 링크 두께
            link_shape: "rectangle" or "ellipse"
            safety_margin: 안전 여유 거리
            
        Returns:
            충돌 지점들의 리스트
        """
        
        collision_points = []
        effective_width = link_width + 2 * safety_margin
        
        # 링크 방향 벡터
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        link_length = math.sqrt(dx*dx + dy*dy)
        
        if link_length < 1e-6:  # 링크 길이가 너무 작으면 점으로 처리
            return self._check_point_collision(start_pos, effective_width/2)
        
        # 링크 방향 단위벡터
        ux = dx / link_length
        uy = dy / link_length
        
        # 링크 수직 방향 단위벡터
        vx = -uy
        vy = ux
        
        # 모든 환경 포인트에 대해 충돌 검사
        for point in self.environment_points:
            px, py = point[0], point[1]
            
            if link_shape == "rectangle":
                if self._point_in_rectangle(px, py, start_pos, end_pos, effective_width):
                    collision_points.append((px, py))
            else:  # ellipse
                if self._point_in_ellipse(px, py, start_pos, end_pos, effective_width):
                    collision_points.append((px, py))
        
        return collision_points
    
    def _point_in_rectangle(self, px: float, py: float, 
                           start_pos: Tuple[float, float], 
                           end_pos: Tuple[float, float],
                           width: float) -> bool:
        """점이 직사각형 링크 내부에 있는지 확인"""
        
        x1, y1 = start_pos
        x2, y2 = end_pos
        
        # 링크를 로컬 좌표계로 변환
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        
        if length < 1e-6:
            return False
        
        # 회전 변환 (링크가 x축과 평행하도록)
        cos_theta = dx / length
        sin_theta = dy / length
        
        # 점을 로컬 좌표계로 변환
        local_x = (px - x1) * cos_theta + (py - y1) * sin_theta
        local_y = -(px - x1) * sin_theta + (py - y1) * cos_theta
        
        # 직사각형 경계 확인
        return (0 <= local_x <= length and 
                -width/2 <= local_y <= width/2)
    
    def _point_in_ellipse(self, px: float, py: float,
                         start_pos: Tuple[float, float], 
                         end_pos: Tuple[float, float],
                         width: float) -> bool:
        """점이 타원형 링크 내부에 있는지 확인"""
        
        x1, y1 = start_pos
        x2, y2 = end_pos
        
        # 타원의 중심
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        # 타원의 장축 (링크 길이)
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        
        if length < 1e-6:
            # 길이가 0이면 원으로 처리
            dist = math.sqrt((px - cx)**2 + (py - cy)**2)
            return dist <= width/2
        
        # 타원 방정식을 위한 좌표 변환
        cos_theta = dx / length
        sin_theta = dy / length
        
        # 점을 타원 중심 기준 로컬 좌표계로 변환
        local_x = (px - cx) * cos_theta + (py - cy) * sin_theta
        local_y = -(px - cx) * sin_theta + (py - cy) * cos_theta
        
        # 타원 방정식: (x/a)^2 + (y/b)^2 <= 1
        a = length / 2  # 장축
        b = width / 2   # 단축
        
        return (local_x/a)**2 + (local_y/b)**2 <= 1
    
    def _check_point_collision(self, center: Tuple[float, float], 
                              radius: float) -> List[Tuple[float, float]]:
        """점 주변의 원형 영역과 충돌 검사"""
        
        collision_points = []
        cx, cy = center
        
        for point in self.environment_points:
            px, py = point[0], point[1]
            dist = math.sqrt((px - cx)**2 + (py - cy)**2)
            
            if dist <= radius:
                collision_points.append((px, py))
        
        return collision_points
    
    def check_multiple_poses(self, poses: List[List[float]], robot_id: int,
                           safety_margin: float = 0.05) -> List[CollisionResult]:
        """
        여러 포즈에 대한 일괄 충돌 검사
        
        Args:
            poses: 포즈 리스트
            robot_id: 로봇 ID
            safety_margin: 안전 여유 거리
            
        Returns:
            각 포즈에 대한 충돌 검사 결과 리스트
        """
        
        results = []
        
        for i, pose in enumerate(poses):
            result = self.check_collision(pose, robot_id, safety_margin)
            results.append(result)
            
            # 진행상황 출력
            if (i + 1) % max(1, len(poses) // 10) == 0:
                collision_count = sum(1 for r in results if r.is_collision)
                print(f"Checked {i+1}/{len(poses)} poses, {collision_count} collisions found")
        
        return results
    
    def get_collision_free_poses(self, poses: List[List[float]], robot_id: int,
                                safety_margin: float = 0.05) -> List[List[float]]:
        """
        충돌 없는 포즈들만 필터링
        
        Args:
            poses: 입력 포즈 리스트
            robot_id: 로봇 ID
            safety_margin: 안전 여유 거리
            
        Returns:
            충돌 없는 포즈들의 리스트
        """
        
        collision_results = self.check_multiple_poses(poses, robot_id, safety_margin)
        
        collision_free_poses = []
        for pose, result in zip(poses, collision_results):
            if not result.is_collision:
                collision_free_poses.append(pose)
        
        collision_count = len(poses) - len(collision_free_poses)
        print(f"Filtered poses: {len(collision_free_poses)} collision-free out of {len(poses)} total")
        print(f"Collision rate: {collision_count/len(poses)*100:.1f}%")
        
        return collision_free_poses
    
    def print_collision_summary(self, result: CollisionResult):
        """충돌 검사 결과 요약 출력"""
        
        if result.is_collision:
            print(f"COLLISION DETECTED!")
            print(f"  Colliding links: {result.colliding_links}")
            print(f"  Number of collision points: {result.num_collision_points}")
        else:
            print("No collision detected.")


if __name__ == "__main__":
    # 간단한 테스트
    print("충돌 감지 테스트 시작...")
    
    test_ply = "data/pointcloud/circles_only/circles_only.ply"
    
    # 포즈 생성기와 충돌 검사기 초기화
    pose_generator = RandomPoseGenerator(seed=42)
    detector = CollisionDetector(pose_generator)
    
    if detector.load_environment(test_ply):
        print("\nGenerating test poses...")
        
        # 테스트 포즈 생성
        test_poses = pose_generator.generate_multiple_poses(
            robot_id=0, 
            num_poses=20,
            workspace_bounds=(0, 10, 0, 8)
        )
        
        if test_poses:
            print(f"\nTesting collision detection on {len(test_poses)} poses...")
            
            # 충돌 검사 수행
            results = detector.check_multiple_poses(test_poses, robot_id=0)
            
            # 결과 분석
            collision_count = sum(1 for r in results if r.is_collision)
            print(f"\nCollision Summary:")
            print(f"  Total poses tested: {len(results)}")
            print(f"  Collisions found: {collision_count}")
            print(f"  Collision rate: {collision_count/len(results)*100:.1f}%")
            
            # 첫 번째 충돌 상세 정보
            for i, result in enumerate(results):
                if result.is_collision:
                    print(f"\nFirst collision (pose {i}):")
                    detector.print_collision_summary(result)
                    break
            
            # 충돌 없는 포즈 필터링 테스트
            collision_free = detector.get_collision_free_poses(test_poses, robot_id=0)
            print(f"\nFiltered to {len(collision_free)} collision-free poses")
    
    else:
        print(f"Could not load test environment: {test_ply}")
        print("Please ensure the PLY file exists for testing.")
    
    print("\nTesting completed!") 