#!/usr/bin/env python3
"""
Random Pose Generator
로봇 geometry 정보를 바탕으로 유효한 랜덤 포즈를 생성하는 모듈

포즈: 3관절 로봇의 각 관절 각도 [theta1, theta2, theta3] (라디안)
"""

import numpy as np
import math
import random
import yaml
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass


@dataclass
class RobotGeometry:
    """로봇 기하학적 정보"""
    robot_id: int
    name: str
    link_shape: str  # "rectangle" or "ellipse"
    link_lengths: List[float]  # [L1, L2, L3]
    link_widths: List[float]   # [W1, W2, W3]
    max_reach: float
    description: str


class RandomPoseGenerator:
    """랜덤 포즈 생성기"""
    
    def __init__(self, config_file: str = "config.yaml", seed: Optional[int] = None):
        """
        Args:
            config_file: config.yaml 파일 경로
            seed: 랜덤 시드
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        self.robot_geometries = self._load_robot_geometries(config_file)
        
    def _load_robot_geometries(self, config_file: str) -> Dict[int, RobotGeometry]:
        """config.yaml에서 로봇 geometry 정보 로드"""
        
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        geometries = {}
        
        if 'robot_geometries' in config:
            for robot_id, robot_config in config['robot_geometries'].items():
                geometries[int(robot_id)] = RobotGeometry(
                    robot_id=int(robot_id),
                    name=robot_config['name'],
                    link_shape=robot_config['link_shape'],
                    link_lengths=robot_config['link_lengths'],
                    link_widths=robot_config['link_widths'],
                    max_reach=robot_config['max_reach'],
                    description=robot_config['description']
                )
        
        return geometries
    
    def get_robot_geometry(self, robot_id: int) -> Optional[RobotGeometry]:
        """특정 로봇 ID의 geometry 정보 반환"""
        return self.robot_geometries.get(robot_id)
    
    def list_available_robots(self) -> List[int]:
        """사용 가능한 로봇 ID 목록"""
        return list(self.robot_geometries.keys())
    
    def generate_random_pose(self, robot_id: int, 
                           workspace_bounds: Tuple[float, float, float, float] = (0, 10, 0, 8),
                           joint_limits: Optional[List[Tuple[float, float]]] = None) -> Optional[List[float]]:
        """
        특정 로봇 geometry에 대한 랜덤 포즈 생성
        
        Args:
            robot_id: 로봇 ID (0-5)
            workspace_bounds: 작업공간 경계 (min_x, max_x, min_y, max_y)
            joint_limits: 관절 제한 [(min1, max1), (min2, max2), (min3, max3)] (라디안)
                         None이면 기본값 사용
            
        Returns:
            [theta1, theta2, theta3] 관절 각도 리스트 (라디안) 또는 None (실패시)
        """
        
        geometry = self.get_robot_geometry(robot_id)
        if geometry is None:
            print(f"Error: Robot ID {robot_id} not found")
            return None
        
        # 기본 관절 제한 설정
        if joint_limits is None:
            joint_limits = [
                (-math.pi, math.pi),    # 첫 번째 관절: 360도 회전
                (-math.pi/2, math.pi/2), # 두 번째 관절: 180도 
                (-math.pi/2, math.pi/2)  # 세 번째 관절: 180도
            ]
        
        max_attempts = 1000
        min_x, max_x, min_y, max_y = workspace_bounds
        
        for attempt in range(max_attempts):
            # 랜덤 관절 각도 생성
            pose = []
            for (min_angle, max_angle) in joint_limits:
                angle = random.uniform(min_angle, max_angle)
                pose.append(angle)
            
            # End-effector 위치 계산
            end_pos = self.forward_kinematics(pose, geometry)
            
            # 작업공간 내부에 있는지 확인
            if min_x <= end_pos[0] <= max_x and min_y <= end_pos[1] <= max_y:
                # 자기 충돌 확인 (간단한 체크)
                if not self._check_self_collision(pose, geometry):
                    return pose
        
        print(f"Warning: Failed to generate valid pose after {max_attempts} attempts")
        return None
    
    def generate_multiple_poses(self, robot_id: int, num_poses: int, 
                               workspace_bounds: Tuple[float, float, float, float] = (0, 10, 0, 8),
                               joint_limits: Optional[List[Tuple[float, float]]] = None) -> List[List[float]]:
        """
        여러 개의 랜덤 포즈 생성
        
        Args:
            robot_id: 로봇 ID
            num_poses: 생성할 포즈 개수
            workspace_bounds: 작업공간 경계
            joint_limits: 관절 제한
            
        Returns:
            생성된 포즈들의 리스트
        """
        
        poses = []
        
        for i in range(num_poses):
            pose = self.generate_random_pose(robot_id, workspace_bounds, joint_limits)
            if pose is not None:
                poses.append(pose)
            
            # 진행상황 출력
            if (i + 1) % max(1, num_poses // 10) == 0:
                print(f"Generated {len(poses)}/{i+1} valid poses...")
        
        print(f"Generated {len(poses)} valid poses out of {num_poses} attempts")
        return poses
    
    def forward_kinematics(self, pose: List[float], geometry: RobotGeometry) -> Tuple[float, float]:
        """
        순기구학: 관절 각도에서 end-effector 위치 계산
        
        Args:
            pose: [theta1, theta2, theta3] 관절 각도 (라디안)
            geometry: 로봇 geometry 정보
            
        Returns:
            (x, y) end-effector 위치
        """
        
        theta1, theta2, theta3 = pose
        L1, L2, L3 = geometry.link_lengths
        
        # 3-링크 로봇 순기구학
        # 베이스는 (0, 0)에 위치
        x = (L1 * math.cos(theta1) + 
             L2 * math.cos(theta1 + theta2) + 
             L3 * math.cos(theta1 + theta2 + theta3))
        
        y = (L1 * math.sin(theta1) + 
             L2 * math.sin(theta1 + theta2) + 
             L3 * math.sin(theta1 + theta2 + theta3))
        
        return (x, y)
    
    def get_all_link_positions(self, pose: List[float], geometry: RobotGeometry) -> List[Tuple[float, float]]:
        """
        모든 링크의 끝점 위치 계산 (충돌 검사용)
        
        Args:
            pose: 관절 각도
            geometry: 로봇 geometry
            
        Returns:
            [(x0, y0), (x1, y1), (x2, y2), (x3, y3)] 
            - (x0, y0): 베이스 위치
            - (x1, y1): 첫 번째 링크 끝
            - (x2, y2): 두 번째 링크 끝  
            - (x3, y3): 세 번째 링크 끝 (end-effector)
        """
        
        theta1, theta2, theta3 = pose
        L1, L2, L3 = geometry.link_lengths
        
        # 베이스 위치
        positions = [(0.0, 0.0)]
        
        # 첫 번째 링크 끝
        x1 = L1 * math.cos(theta1)
        y1 = L1 * math.sin(theta1)
        positions.append((x1, y1))
        
        # 두 번째 링크 끝
        x2 = x1 + L2 * math.cos(theta1 + theta2)
        y2 = y1 + L2 * math.sin(theta1 + theta2)
        positions.append((x2, y2))
        
        # 세 번째 링크 끝 (end-effector)
        x3 = x2 + L3 * math.cos(theta1 + theta2 + theta3)
        y3 = y2 + L3 * math.sin(theta1 + theta2 + theta3)
        positions.append((x3, y3))
        
        return positions
    
    def _check_self_collision(self, pose: List[float], geometry: RobotGeometry) -> bool:
        """
        간단한 자기 충돌 검사
        
        Args:
            pose: 관절 각도
            geometry: 로봇 geometry
            
        Returns:
            True if 충돌 존재, False if 충돌 없음
        """
        
        positions = self.get_all_link_positions(pose, geometry)
        
        # 간단한 거리 기반 충돌 검사
        # 인접하지 않은 링크 간 최소 거리 확인
        for i in range(len(positions)):
            for j in range(i + 2, len(positions)):  # 인접하지 않은 링크만
                if i == 0 and j == len(positions) - 1:  # 베이스와 끝점은 제외
                    continue
                
                pos1 = positions[i]
                pos2 = positions[j]
                distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                
                # 최소 허용 거리 (링크 두께 고려)
                min_distance = max(geometry.link_widths) * 1.5
                
                if distance < min_distance:
                    return True  # 충돌 발견
        
        return False  # 충돌 없음
    
    def print_robot_info(self, robot_id: int):
        """로봇 정보 출력"""
        geometry = self.get_robot_geometry(robot_id)
        if geometry is None:
            print(f"Robot ID {robot_id} not found")
            return
        
        print(f"Robot ID {robot_id}: {geometry.name}")
        print(f"  Description: {geometry.description}")
        print(f"  Link Shape: {geometry.link_shape}")
        print(f"  Link Lengths: {geometry.link_lengths}")
        print(f"  Link Widths: {geometry.link_widths}")
        print(f"  Max Reach: {geometry.max_reach}")


if __name__ == "__main__":
    # 테스트 실행
    print("Testing Random Pose Generator...")
    
    generator = RandomPoseGenerator(seed=42)
    
    # 사용 가능한 로봇 목록
    print(f"Available robots: {generator.list_available_robots()}")
    
    # 로봇 0에 대한 정보 출력
    generator.print_robot_info(0)
    
    # 랜덤 포즈 생성 테스트
    print("\nGenerating random poses for Robot 0...")
    poses = generator.generate_multiple_poses(robot_id=0, num_poses=10)
    
    # 첫 번째 포즈 분석
    if poses:
        pose = poses[0]
        geometry = generator.get_robot_geometry(0)
        end_pos = generator.forward_kinematics(pose, geometry)
        all_positions = generator.get_all_link_positions(pose, geometry)
        
        print(f"\nFirst generated pose:")
        print(f"  Joint angles: {[math.degrees(angle) for angle in pose]} degrees")
        print(f"  End-effector position: ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
        print(f"  All link positions: {[(round(x, 2), round(y, 2)) for x, y in all_positions]}")
    
    print("\nTesting completed!") 