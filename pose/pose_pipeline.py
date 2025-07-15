#!/usr/bin/env python3
"""
Pose Pipeline
Random Pose Generator와 Collision Detector를 통합하여 사용하는 파이프라인

사용법:
    python pose_pipeline.py <environment.ply> <robot_id> [num_poses]
    
예시:
    python pose_pipeline.py pointcloud/data/circles_only/circles_only.ply 0 100
"""

import sys
import os
import argparse
import time
from typing import List, Tuple

try:
    from .random_pose_generator import RandomPoseGenerator
    from .collision_detector import CollisionDetector
except ImportError:
    from random_pose_generator import RandomPoseGenerator
    from collision_detector import CollisionDetector


class PosePipeline:
    """포즈 생성 및 충돌 검사 파이프라인"""
    
    def __init__(self, config_file: str = "config.yaml", seed: int = None):
        """
        Args:
            config_file: config.yaml 파일 경로
            seed: 랜덤 시드
        """
        self.pose_generator = RandomPoseGenerator(config_file, seed)
        self.collision_detector = CollisionDetector(self.pose_generator)
        
    def generate_collision_free_poses(self, ply_file: str, robot_id: int, 
                                    num_target_poses: int = 100,
                                    max_generation_attempts: int = 1000,
                                    safety_margin: float = 0.05) -> Tuple[List[List[float]], dict]:
        """
        충돌 없는 포즈들을 생성하는 완전한 파이프라인
        
        Args:
            ply_file: 환경 PLY 파일 경로
            robot_id: 로봇 ID
            num_target_poses: 목표 충돌 없는 포즈 개수
            max_generation_attempts: 최대 생성 시도 횟수
            safety_margin: 안전 여유 거리
            
        Returns:
            (collision_free_poses, statistics)
        """
        
        print(f"🚀 Starting Pose Pipeline...")
        print(f"   Environment: {ply_file}")
        print(f"   Robot ID: {robot_id}")
        print(f"   Target poses: {num_target_poses}")
        
        start_time = time.time()
        
        # 1. 환경 로드
        print(f"\n📁 Loading environment...")
        if not self.collision_detector.load_environment(ply_file):
            raise Exception(f"Failed to load environment: {ply_file}")
        
        # 2. 로봇 정보 확인
        print(f"\n🤖 Robot Information:")
        self.pose_generator.print_robot_info(robot_id)
        
        # 3. 포즈 생성 및 충돌 검사
        print(f"\n🎯 Generating poses...")
        
        collision_free_poses = []
        total_generated = 0
        total_checked = 0
        collision_count = 0
        
        batch_size = min(100, max_generation_attempts // 10)
        
        while len(collision_free_poses) < num_target_poses and total_generated < max_generation_attempts:
            # 배치 단위로 포즈 생성
            remaining_attempts = max_generation_attempts - total_generated
            current_batch_size = min(batch_size, remaining_attempts)
            
            # 포즈 생성
            batch_poses = self.pose_generator.generate_multiple_poses(
                robot_id=robot_id,
                num_poses=current_batch_size,
                workspace_bounds=self.collision_detector.environment_bounds
            )
            
            total_generated += current_batch_size
            
            if not batch_poses:
                print(f"Warning: No valid poses generated in batch")
                continue
            
            # 충돌 검사
            collision_results = self.collision_detector.check_multiple_poses(
                batch_poses, robot_id, safety_margin
            )
            
            total_checked += len(batch_poses)
            
            # 충돌 없는 포즈 수집
            for pose, result in zip(batch_poses, collision_results):
                if not result.is_collision:
                    collision_free_poses.append(pose)
                    if len(collision_free_poses) >= num_target_poses:
                        break
                else:
                    collision_count += 1
            
            # 진행상황 출력
            print(f"   Progress: {len(collision_free_poses)}/{num_target_poses} poses, "
                  f"{total_checked} checked, {collision_count} collisions")
        
        end_time = time.time()
        duration = end_time - start_time
        
        # 4. 결과 통계
        statistics = {
            'target_poses': num_target_poses,
            'achieved_poses': len(collision_free_poses),
            'total_generated': total_generated,
            'total_checked': total_checked,
            'collision_count': collision_count,
            'collision_rate': collision_count / total_checked if total_checked > 0 else 0,
            'success_rate': len(collision_free_poses) / total_checked if total_checked > 0 else 0,
            'generation_time': duration,
            'poses_per_second': len(collision_free_poses) / duration if duration > 0 else 0,
            'robot_id': robot_id,
            'environment_file': ply_file,
            'safety_margin': safety_margin
        }
        
        # 결과 출력
        self._print_pipeline_results(statistics)
        
        return collision_free_poses[:num_target_poses], statistics
    
    def _print_pipeline_results(self, stats: dict):
        """파이프라인 결과 출력"""
        
        print(f"\n📊 Pipeline Results:")
        print(f"   ✅ Generated: {stats['achieved_poses']}/{stats['target_poses']} collision-free poses")
        print(f"   ⏱️  Time: {stats['generation_time']:.1f}s ({stats['poses_per_second']:.1f} poses/sec)")
        print(f"   🎲 Total generated: {stats['total_generated']} poses")
        print(f"   🔍 Total checked: {stats['total_checked']} poses")
        print(f"   💥 Collisions: {stats['collision_count']} ({stats['collision_rate']*100:.1f}%)")
        print(f"   ✨ Success rate: {stats['success_rate']*100:.1f}%")
    
    def save_poses_to_file(self, poses: List[List[float]], filename: str, 
                          robot_id: int, statistics: dict = None):
        """포즈들을 파일로 저장"""
        
        import json
        
        data = {
            'robot_id': robot_id,
            'num_poses': len(poses),
            'poses': poses,
            'statistics': statistics,
            'format': 'joint_angles_radians'
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"💾 Saved {len(poses)} poses to {filename}")
    
    def analyze_pose_distribution(self, poses: List[List[float]], robot_id: int):
        """포즈 분포 분석"""
        
        if not poses:
            print("No poses to analyze")
            return
        
        import numpy as np
        
        poses_array = np.array(poses)
        
        print(f"\n📈 Pose Distribution Analysis:")
        print(f"   Number of poses: {len(poses)}")
        
        for i in range(poses_array.shape[1]):
            joint_angles = poses_array[:, i]
            mean_angle = np.mean(joint_angles)
            std_angle = np.std(joint_angles)
            min_angle = np.min(joint_angles)
            max_angle = np.max(joint_angles)
            
            print(f"   Joint {i+1}: mean={np.degrees(mean_angle):.1f}°, "
                  f"std={np.degrees(std_angle):.1f}°, "
                  f"range=[{np.degrees(min_angle):.1f}°, {np.degrees(max_angle):.1f}°]")
        
        # End-effector 위치 분포
        geometry = self.pose_generator.get_robot_geometry(robot_id)
        if geometry:
            end_positions = []
            for pose in poses:
                end_pos = self.pose_generator.forward_kinematics(pose, geometry)
                end_positions.append(end_pos)
            
            end_positions = np.array(end_positions)
            
            print(f"   End-effector positions:")
            print(f"     X: mean={np.mean(end_positions[:, 0]):.2f}, "
                  f"std={np.std(end_positions[:, 0]):.2f}, "
                  f"range=[{np.min(end_positions[:, 0]):.2f}, {np.max(end_positions[:, 0]):.2f}]")
            print(f"     Y: mean={np.mean(end_positions[:, 1]):.2f}, "
                  f"std={np.std(end_positions[:, 1]):.2f}, "
                  f"range=[{np.min(end_positions[:, 1]):.2f}, {np.max(end_positions[:, 1]):.2f}]")


def parse_arguments():
    """명령행 인수 파싱"""
    
    parser = argparse.ArgumentParser(description='Generate collision-free robot poses')
    
    parser.add_argument('ply_file', type=str, 
                       help='Path to PLY environment file')
    parser.add_argument('robot_id', type=int, 
                       help='Robot ID (0-5)')
    parser.add_argument('--num_poses', type=int, default=100,
                       help='Number of collision-free poses to generate (default: 100)')
    parser.add_argument('--max_attempts', type=int, default=1000,
                       help='Maximum generation attempts (default: 1000)')
    parser.add_argument('--safety_margin', type=float, default=0.05,
                       help='Safety margin for collision detection (default: 0.05)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output JSON file for saving poses')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')
    parser.add_argument('--analyze', action='store_true',
                       help='Perform pose distribution analysis')
    
    return parser.parse_args()


def main():
    """메인 함수"""
    
    args = parse_arguments()
    
    # 파일 존재 확인
    if not os.path.exists(args.ply_file):
        print(f"Error: PLY file not found: {args.ply_file}")
        return 1
    
    # 로봇 ID 확인
    if args.robot_id < 0 or args.robot_id > 5:
        print(f"Error: Invalid robot ID: {args.robot_id}. Must be 0-5.")
        return 1
    
    try:
        # 파이프라인 실행
        pipeline = PosePipeline(seed=args.seed)
        
        collision_free_poses, statistics = pipeline.generate_collision_free_poses(
            ply_file=args.ply_file,
            robot_id=args.robot_id,
            num_target_poses=args.num_poses,
            max_generation_attempts=args.max_attempts,
            safety_margin=args.safety_margin
        )
        
        # 결과 저장
        if args.output:
            pipeline.save_poses_to_file(collision_free_poses, args.output, 
                                      args.robot_id, statistics)
        
        # 분석
        if args.analyze:
            pipeline.analyze_pose_distribution(collision_free_poses, args.robot_id)
        
        if len(collision_free_poses) < args.num_poses:
            print(f"\n⚠️  Warning: Only generated {len(collision_free_poses)} out of {args.num_poses} requested poses")
            return 1
        else:
            print(f"\n🎉 Success: Generated all {len(collision_free_poses)} requested poses!")
            return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main()) 