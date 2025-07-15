#!/usr/bin/env python3
"""
Batch Pose Generator
특정 환경-로봇 geometry 쌍에 대해 collision-free pose를 대량 생성하여 저장

사용법:
    python batch_pose_generator.py <environment> <robot_id> --num_poses <n>
    
예시:
    # 기본 환경에 대해
    python batch_pose_generator.py circles_only 0 --num_poses 50
    
    # circle_envs 환경에 대해
    python batch_pose_generator.py circle_envs_10k/env_0001 2 --num_poses 100
"""

import os
import sys
import json
import argparse
import time
from pathlib import Path
from typing import List, Tuple, Dict, Any

try:
    from .pose_pipeline import PosePipeline
except ImportError:
    from pose_pipeline import PosePipeline


class BatchPoseGenerator:
    """배치 포즈 생성기"""
    
    def __init__(self, config_file: str = "config.yaml", seed: int = None):
        """
        Args:
            config_file: config.yaml 파일 경로
            seed: 랜덤 시드
        """
        self.pipeline = PosePipeline(config_file, seed)
        self.data_dir = Path("data/pose")
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
    def generate_and_save_poses(self, 
                               environment: str, 
                               robot_id: int, 
                               num_poses: int = 100,
                               max_attempts: int = 1000,
                               safety_margin: float = 0.05) -> str:
        """
        특정 환경-로봇 쌍에 대해 pose 생성 및 저장
        
        Args:
            environment: 환경 이름 (예: 'circles_only', 'circle_envs_10k/env_0001')
            robot_id: 로봇 ID (0-5)
            num_poses: 생성할 pose 개수
            max_attempts: 최대 시도 횟수
            safety_margin: 안전 여유거리
            
        Returns:
            저장된 파일 경로
        """
        
        # Environment 경로 해석
        possible_paths = [
            # 예: circle_envs_10k/env_0001 -> data/pointcloud/circle_envs_10k/env_0001.ply
            f"data/pointcloud/{environment}.ply",
            
            # 단일 파일명 처리
            f"data/pointcloud/{environment}/{environment}.ply",
            f"data/pointcloud/{environment}.ply",
            environment  # 직접 경로인 경우
        ]
        
        ply_file = None
        for path in possible_paths:
            if os.path.exists(path):
                ply_file = path
                break
        
        if not ply_file:
            raise ValueError(f"Environment file not found: {environment}")
        
        print(f"🚀 Generating poses for {environment} with Robot {robot_id}")
        print(f"   Target poses: {num_poses}")
        print(f"   PLY file: {ply_file}")
        
        # 포즈 생성
        start_time = time.time()
        poses, statistics = self.pipeline.generate_collision_free_poses(
            ply_file=ply_file,
            robot_id=robot_id,
            num_target_poses=num_poses,
            max_generation_attempts=max_attempts,
            safety_margin=safety_margin
        )
        generation_time = time.time() - start_time
        
        # 환경 메타데이터 수집
        env_metadata = self._extract_environment_metadata(ply_file, environment)
        
        # 로봇 메타데이터 수집
        robot_metadata = self._extract_robot_metadata(robot_id)
        
        # 저장할 데이터 구성
        pose_data = {
            'environment': {
                'name': environment,
                'ply_file': ply_file,
                'metadata': env_metadata
            },
            'robot': {
                'id': robot_id,
                'metadata': robot_metadata
            },
            'poses': {
                'data': poses,
                'count': len(poses),
                'format': 'joint_angles_radians'
            },
            'generation_info': {
                'target_poses': num_poses,
                'achieved_poses': len(poses),
                'generation_time': generation_time,
                'safety_margin': safety_margin,
                'max_attempts': max_attempts,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            },
            'statistics': statistics
        }
        
        # 파일명 생성 및 저장
        output_file = self._generate_filename(environment, robot_id)
        self._save_pose_data(pose_data, output_file)
        
        return output_file
    
    def _resolve_environment_path(self, environment: str) -> str:
        """환경 이름을 PLY 파일 경로로 변환"""
        
        # circle_envs_10k 형태 처리
        if 'circle_envs' in environment and '/' in environment:
            # 예: circle_envs_10k/env_0001 -> pointcloud/data/circle_envs_10k/env_0001.ply
            env_path = f"pointcloud/data/{environment}.ply"
            if os.path.exists(env_path):
                return env_path
        
        # 기본 환경들 처리
        base_paths = [
            f"pointcloud/data/{environment}/{environment}.ply",
            f"pointcloud/data/{environment}.ply",
            environment  # 직접 경로인 경우
        ]
        
        for path in base_paths:
            if os.path.exists(path):
                return path
        
        return None
    
    def _extract_environment_metadata(self, ply_file: str, env_name: str) -> Dict[str, Any]:
        """환경 메타데이터 추출"""
        metadata = {
            'name': env_name,
            'ply_file': ply_file
        }
        
        # JSON 메타데이터 파일 찾기
        json_paths = [
            ply_file.replace('.ply', '_meta.json'),
            os.path.join(os.path.dirname(ply_file), f"{os.path.basename(ply_file).replace('.ply', '')}_meta.json")
        ]
        
        for json_path in json_paths:
            if os.path.exists(json_path):
                try:
                    with open(json_path, 'r') as f:
                        json_metadata = json.load(f)
                        metadata.update(json_metadata)
                        break
                except Exception as e:
                    print(f"Warning: Could not load metadata from {json_path}: {e}")
        
        # PLY 파일에서 기본 정보 추출
        try:
            with open(ply_file, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if line.startswith('element vertex'):
                        metadata['num_points'] = int(line.split()[-1])
                        break
        except Exception as e:
            print(f"Warning: Could not extract PLY info: {e}")
        
        return metadata
    
    def _extract_robot_metadata(self, robot_id: int) -> Dict[str, Any]:
        """로봇 메타데이터 추출"""
        robot_geometry = self.pipeline.pose_generator.get_robot_geometry(robot_id)
        
        if robot_geometry:
            return {
                'id': robot_id,
                'description': robot_geometry.description,
                'link_shape': robot_geometry.link_shape,
                'link_lengths': robot_geometry.link_lengths,
                'link_widths': robot_geometry.link_widths,
                'max_reach': robot_geometry.max_reach
            }
        else:
            return {
                'id': robot_id,
                'description': 'Unknown',
                'link_shape': 'Unknown',
                'link_lengths': [],
                'link_widths': [],
                'max_reach': 0
            }
    
    def _generate_filename(self, environment: str, robot_id: int) -> str:
        """파일명 생성"""
        
        # circle_envs 형태 처리
        if 'circle_envs' in environment and '/' in environment:
            # circle_envs_10k/env_0001 -> circle_envs/env_0001_geo_0_poses.json
            parts = environment.split('/')
            env_folder = parts[0].replace('_10k', '')  # circle_envs_10k -> circle_envs
            env_name = parts[1]  # env_0001
            
            folder_path = self.data_dir / env_folder
            folder_path.mkdir(parents=True, exist_ok=True)
            
            filename = f"{env_name}_geo_{robot_id}_poses.json"
            return str(folder_path / filename)
        
        # 기본 환경들 처리
        else:
            filename = f"{environment}_geo_{robot_id}_poses.json"
            return str(self.data_dir / filename)
    
    def _save_pose_data(self, data: Dict[str, Any], filename: str) -> None:
        """포즈 데이터 저장"""
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        achieved = data['poses']['count']
        target = data['generation_info']['target_poses']
        print(f"💾 Saved {achieved}/{target} poses to {filename}")


def parse_arguments():
    """명령행 인수 파싱"""
    parser = argparse.ArgumentParser(description='Generate collision-free poses for environment-robot pairs')
    
    parser.add_argument('environment', type=str,
                       help='Environment name (e.g., circles_only, circle_envs_10k/env_0001)')
    parser.add_argument('robot_id', type=int,
                       help='Robot ID (0-5)')
    parser.add_argument('--num_poses', type=int, default=100,
                       help='Number of poses to generate (default: 100)')
    parser.add_argument('--max_attempts', type=int, default=1000,
                       help='Maximum generation attempts (default: 1000)')
    parser.add_argument('--safety_margin', type=float, default=0.05,
                       help='Safety margin for collision detection (default: 0.05)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')
    
    return parser.parse_args()


def main():
    """메인 함수"""
    args = parse_arguments()
    
    # 로봇 ID 검증
    if args.robot_id < 0 or args.robot_id > 5:
        print(f"Error: Invalid robot ID: {args.robot_id}. Must be 0-5.")
        return 1
    
    try:
        # 생성기 초기화
        generator = BatchPoseGenerator(seed=args.seed)
        
        # 포즈 생성 및 저장
        output_file = generator.generate_and_save_poses(
            environment=args.environment,
            robot_id=args.robot_id,
            num_poses=args.num_poses,
            max_attempts=args.max_attempts,
            safety_margin=args.safety_margin
        )
        
        print(f"\n🎉 Successfully saved poses to: {output_file}")
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main()) 