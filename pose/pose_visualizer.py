#!/usr/bin/env python3
"""
Pose Visualizer
저장된 pose JSON 파일들을 환경과 함께 시각화

사용법:
    python pose_visualizer.py <pose_json_file>
    
예시:
    python pose_visualizer.py data/pose/circles_only_geo_0_poses.json
    python pose_visualizer.py data/pose/circle_envs/env_0001_geo_2_poses.json --save_image
"""

import os
import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Dict, Any, Tuple

try:
    from .random_pose_generator import RandomPoseGenerator
except ImportError:
    from random_pose_generator import RandomPoseGenerator


class PoseVisualizer:
    """포즈 시각화기"""
    
    def __init__(self, config_file: str = "config.yaml"):
        """
        Args:
            config_file: config.yaml 파일 경로
        """
        self.pose_generator = RandomPoseGenerator(config_file)
        
    def visualize_poses_from_file(self, 
                                json_file: str, 
                                save_image: bool = False,
                                output_file: str = None,
                                show_plot: bool = True) -> None:
        """
        JSON 파일에서 pose들을 로드하여 시각화
        
        Args:
            json_file: pose JSON 파일 경로
            save_image: 이미지 저장 여부
            output_file: 출력 이미지 파일명
            show_plot: 플롯 표시 여부
        """
        
        # JSON 파일 로드
        pose_data = self._load_pose_data(json_file)
        
        # 환경 데이터 로드
        environment_points = self._load_environment_data(pose_data)
        
        # 시각화
        self._create_visualization(pose_data, environment_points, save_image, output_file, show_plot)
    
    def _load_pose_data(self, json_file: str) -> Dict[str, Any]:
        """JSON 파일에서 pose 데이터 로드"""
        
        if not os.path.exists(json_file):
            raise FileNotFoundError(f"Pose file not found: {json_file}")
        
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        print(f"📂 Loaded pose data from: {json_file}")
        print(f"   Environment: {data['environment']['name']}")
        print(f"   Robot: {data['robot']['metadata']['description']}")
        print(f"   Poses: {data['poses']['count']}")
        
        return data
    
    def _load_environment_data(self, pose_data: Dict[str, Any]) -> np.ndarray:
        """환경 PLY 파일에서 포인트 데이터 로드"""
        
        ply_file = pose_data['environment']['ply_file']
        
        if not os.path.exists(ply_file):
            print(f"Warning: PLY file not found: {ply_file}")
            return np.array([])
        
        try:
            points = []
            with open(ply_file, 'r') as f:
                lines = f.readlines()
                
                # 헤더 파싱
                header_end = False
                for i, line in enumerate(lines):
                    if line.strip() == 'end_header':
                        header_end = True
                        data_start = i + 1
                        break
                
                if not header_end:
                    raise ValueError("Invalid PLY format: no end_header found")
                
                # 포인트 데이터 파싱
                for line in lines[data_start:]:
                    if line.strip():
                        coords = line.strip().split()
                        if len(coords) >= 2:
                            x, y = float(coords[0]), float(coords[1])
                            points.append([x, y])
            
            points_array = np.array(points)
            print(f"   Environment points: {len(points_array)}")
            return points_array
            
        except Exception as e:
            print(f"Warning: Could not load environment data: {e}")
            return np.array([])
    
    def _create_visualization(self, 
                            pose_data: Dict[str, Any], 
                            environment_points: np.ndarray,
                            save_image: bool = False,
                            output_file: str = None,
                            show_plot: bool = True) -> None:
        """포즈들과 환경을 시각화"""
        
        # 데이터 추출
        poses = pose_data['poses']['data']
        robot_id = pose_data['robot']['id']
        robot_metadata = pose_data['robot']['metadata']
        env_name = pose_data['environment']['name']
        
        # 로봇 geometry 가져오기
        robot_geometry = self.pose_generator.get_robot_geometry(robot_id)
        if not robot_geometry:
            raise ValueError(f"Could not get robot geometry for ID {robot_id}")
        
        # Figure 설정
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # 환경 시각화
        if len(environment_points) > 0:
            ax.scatter(environment_points[:, 0], environment_points[:, 1], 
                      c='darkblue', alpha=0.6, s=8, label='Environment')
        
        # 로봇 베이스 위치 표시
        ax.plot(0, 0, 'ro', markersize=10, label='Robot Base', zorder=10)
        
        # 각 pose에 대해 로봇 시각화
        colors = plt.cm.viridis(np.linspace(0, 1, len(poses)))  # 포즈별 색상
        
        print(f"🎨 Visualizing {len(poses)} poses...")
        
        for i, pose in enumerate(poses):
            try:
                # Forward kinematics로 링크 위치들 계산
                link_positions = self._calculate_link_positions(pose, robot_geometry)
                
                # 로봇 링크들 시각화
                self._draw_robot_links(ax, link_positions, robot_geometry, colors[i], alpha=0.7)
                
            except Exception as e:
                print(f"Warning: Could not visualize pose {i}: {e}")
        
        # 작업공간 표시
        max_reach = robot_metadata.get('max_reach', 7.5)
        workspace_circle = plt.Circle((0, 0), max_reach, fill=False, color='gray', 
                                     linestyle='--', alpha=0.5, label=f'Max Reach ({max_reach:.1f}m)')
        ax.add_patch(workspace_circle)
        
        # 플롯 설정
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        
        # 로봇 설명을 영어로 변환
        robot_desc_en = self._translate_robot_description(robot_metadata.get("description", "Unknown Robot"))
        
        ax.set_title(f'Pose Visualization: {env_name} - Robot {robot_id}\n'
                    f'{robot_desc_en} - {len(poses)} poses', 
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.legend()
        
        # 축 범위 설정 - 로봇 베이스(0,0)와 모든 pose들이 보이도록
        x_coords = [0.0]  # 로봇 베이스 포함
        y_coords = [0.0]
        
        # 모든 pose의 end-effector 위치 수집
        for pose in poses:
            try:
                link_positions = self._calculate_link_positions(pose, robot_geometry)
                for x, y in link_positions:
                    x_coords.append(x)
                    y_coords.append(y)
            except Exception:
                pass
        
        # 환경 포인트들도 고려
        if len(environment_points) > 0:
            x_coords.extend(environment_points[:, 0])
            y_coords.extend(environment_points[:, 1])
        
        if x_coords and y_coords:
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            
            # 여유 공간 추가
            margin = max(1.0, (x_max - x_min) * 0.1, (y_max - y_min) * 0.1)
            ax.set_xlim(x_min - margin, x_max + margin)
            ax.set_ylim(y_min - margin, y_max + margin)
        else:
            # 기본 범위
            ax.set_xlim(-max_reach - 1, max_reach + 1)
            ax.set_ylim(-max_reach - 1, max_reach + 1)
        
        # 정보 박스 추가
        info_text = self._create_info_text(pose_data)
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        # 이미지 저장
        if save_image:
            # data/results/poses 디렉터리 확인 및 생성
            result_dir = Path("data/results/poses")
            result_dir.mkdir(parents=True, exist_ok=True)
            
            if output_file is None:
                base_name = Path(pose_data['environment']['name']).name
                robot_id = pose_data['robot']['id']
                output_file = f"pose_visualization_{base_name}_robot_{robot_id}.png"
            
            # 출력 파일 경로를 data/results/poses 하위로 설정
            if not output_file.startswith('data/results/poses/'):
                output_file = result_dir / output_file
            
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"💾 Saved visualization to: {output_file}")
        
        # 플롯 표시
        if show_plot:
            plt.show()
        else:
            plt.close()
    
    def _calculate_link_positions(self, pose: List[float], robot_geometry) -> List[Tuple[float, float]]:
        """Forward kinematics로 링크 위치들 계산"""
        
        link_lengths = robot_geometry.link_lengths
        
        positions = [(0.0, 0.0)]  # 베이스 위치
        current_angle = 0.0
        current_x, current_y = 0.0, 0.0
        
        for i, (joint_angle, link_length) in enumerate(zip(pose, link_lengths)):
            current_angle += joint_angle
            
            # 링크 끝 위치 계산
            current_x += link_length * np.cos(current_angle)
            current_y += link_length * np.sin(current_angle)
            
            positions.append((current_x, current_y))
        
        return positions
    
    def _draw_robot_links(self, ax, link_positions: List[Tuple[float, float]], 
                         robot_geometry, color, alpha: float = 0.7) -> None:
        """로봇 링크들을 시각화"""
        
        link_shape = robot_geometry.link_shape
        link_widths = robot_geometry.link_widths
        
        # 링크들을 선으로 연결
        x_coords = [pos[0] for pos in link_positions]
        y_coords = [pos[1] for pos in link_positions]
        
        ax.plot(x_coords, y_coords, color=color, linewidth=3, alpha=alpha)
        
        # 관절 위치 표시
        for i, (x, y) in enumerate(link_positions[:-1]):  # 마지막 end-effector 제외
            ax.plot(x, y, 'o', color=color, markersize=6, alpha=alpha)
        
        # End-effector 특별 표시
        end_x, end_y = link_positions[-1]
        ax.plot(end_x, end_y, 's', color=color, markersize=8, alpha=alpha)
        
        # 링크 두께 시각화 (옵션)
        if link_shape == 'rectangle':
            for i in range(len(link_positions) - 1):
                start_pos = link_positions[i]
                end_pos = link_positions[i + 1]
                width = link_widths[i] if i < len(link_widths) else 0.2
                
                # 링크 방향 벡터
                dx = end_pos[0] - start_pos[0]
                dy = end_pos[1] - start_pos[1]
                length = np.sqrt(dx**2 + dy**2)
                
                if length > 0:
                    # 수직 벡터
                    perp_x = -dy / length * width / 2
                    perp_y = dx / length * width / 2
                    
                    # 사각형 꼭짓점들
                    corners = [
                        (start_pos[0] + perp_x, start_pos[1] + perp_y),
                        (start_pos[0] - perp_x, start_pos[1] - perp_y),
                        (end_pos[0] - perp_x, end_pos[1] - perp_y),
                        (end_pos[0] + perp_x, end_pos[1] + perp_y),
                        (start_pos[0] + perp_x, start_pos[1] + perp_y)  # 닫기
                    ]
                    
                    corner_x = [c[0] for c in corners]
                    corner_y = [c[1] for c in corners]
                    
                    ax.fill(corner_x, corner_y, color=color, alpha=alpha*0.3)
    
    def _translate_robot_description(self, korean_desc: str) -> str:
        """한글 로봇 설명을 영어로 변환"""
        translations = {
            "표준 직사각형 링크 로봇": "Standard Rectangle Robot",
            "확장형 직사각형 링크 로봇": "Extended Rectangle Robot", 
            "컴팩트 직사각형 링크 로봇": "Compact Rectangle Robot",
            "표준 타원형 링크 로봇": "Standard Ellipse Robot",
            "좁은 타원형 링크 로봇": "Narrow Ellipse Robot",
            "넓은 타원형 링크 로봇": "Wide Ellipse Robot"
        }
        return translations.get(korean_desc, korean_desc)

    def _create_info_text(self, pose_data: Dict[str, Any]) -> str:
        """정보 텍스트 생성"""
        
        env_name = pose_data['environment']['name']
        robot_desc = self._translate_robot_description(pose_data['robot']['metadata']['description'])
        poses_count = pose_data['poses']['count']
        target_count = pose_data['generation_info']['target_poses']
        generation_time = pose_data['generation_info']['generation_time']
        
        info_lines = [
            f"Environment: {env_name}",
            f"Robot: {robot_desc}",
            f"Poses: {poses_count}/{target_count}",
            f"Generation Time: {generation_time:.2f}s"
        ]
        
        # 통계 정보 추가
        if 'statistics' in pose_data and pose_data['statistics']:
            stats = pose_data['statistics']
            success_rate = stats.get('success_rate', 0) * 100
            collision_rate = stats.get('collision_rate', 0) * 100
            
            info_lines.extend([
                f"Success Rate: {success_rate:.1f}%",
                f"Collision Rate: {collision_rate:.1f}%"
            ])
        
        return "\n".join(info_lines)


def parse_arguments():
    """명령행 인수 파싱"""
    parser = argparse.ArgumentParser(description='Visualize poses from JSON file')
    
    parser.add_argument('json_file', type=str,
                       help='Path to pose JSON file')
    parser.add_argument('--save_image', action='store_true',
                       help='Save visualization as image')
    parser.add_argument('--output', type=str, default=None,
                       help='Output image filename')
    parser.add_argument('--no_show', action='store_true',
                       help='Don\'t display plot (useful for batch processing)')
    
    return parser.parse_args()


def main():
    """메인 함수"""
    args = parse_arguments()
    
    if not os.path.exists(args.json_file):
        print(f"Error: JSON file not found: {args.json_file}")
        return 1
    
    try:
        # 시각화기 초기화
        visualizer = PoseVisualizer()
        
        # 시각화 실행
        visualizer.visualize_poses_from_file(
            json_file=args.json_file,
            save_image=args.save_image,
            output_file=args.output,
            show_plot=not args.no_show
        )
        
        print(f"\n🎉 Successfully visualized poses from: {args.json_file}")
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main()) 