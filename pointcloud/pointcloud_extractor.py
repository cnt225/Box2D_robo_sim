"""
Pointcloud Extractor Module
환경에서 포인트클라우드를 추출하는 모듈

Box2D 시뮬레이션 환경에서 장애물들을 스캔하여 포인트클라우드로 변환
"""
import numpy as np
import os
from typing import List, Tuple, Optional
import json

class PointcloudExtractor:
    """Box2D 환경에서 포인트클라우드를 추출하는 클래스"""
    
    def __init__(self, resolution: float = 0.05, noise_level: float = 0.01):
        """
        Args:
            resolution: 포인트클라우드 해상도 (미터)
            noise_level: 센서 노이즈 레벨
        """
        self.resolution = resolution
        self.noise_level = noise_level
        self.data_dir = "pointcloud/data"
        
        # 데이터 디렉토리 생성
        os.makedirs(self.data_dir, exist_ok=True)
    
    def extract_from_world(self, world, workspace_bounds: Tuple[float, float, float, float]) -> np.ndarray:
        """
        Box2D 월드에서 포인트클라우드 추출
        
        Args:
            world: Box2D world 객체
            workspace_bounds: (min_x, max_x, min_y, max_y) 작업공간 경계
            
        Returns:
            points: (N, 2) numpy array, [x, y] 좌표
        """
        min_x, max_x, min_y, max_y = workspace_bounds
        points = []
        
        # 각 정적 body (장애물)에 대해 포인트 생성
        for body in world.bodies:
            if body.type == 0:  # staticBody
                for fixture in body.fixtures:
                    obstacle_points = self._sample_fixture_points(body, fixture)
                    points.extend(obstacle_points)
        
        if not points:
            return np.array([]).reshape(0, 2)
        
        points = np.array(points)
        
        # 작업공간 내부 포인트만 필터링
        mask = (points[:, 0] >= min_x) & (points[:, 0] <= max_x) & \
               (points[:, 1] >= min_y) & (points[:, 1] <= max_y)
        points = points[mask]
        
        # 센서 노이즈 추가
        if self.noise_level > 0:
            noise = np.random.normal(0, self.noise_level, points.shape)
            points += noise
        
        return points
    
    def _sample_fixture_points(self, body, fixture) -> List[Tuple[float, float]]:
        """Fixture 경계에서 포인트 샘플링"""
        points = []
        
        if hasattr(fixture.shape, 'vertices'):
            # Polygon fixture
            vertices = []
            for v in fixture.shape.vertices:
                # 로컬 좌표를 월드 좌표로 변환
                world_point = body.GetWorldPoint(v)
                vertices.append((world_point.x, world_point.y))
            
            # 폴리곤 경계를 따라 포인트 샘플링
            points.extend(self._sample_polygon_boundary(vertices))
            
        elif hasattr(fixture.shape, 'radius'):
            # Circle fixture
            center = body.GetWorldPoint((0, 0))
            radius = fixture.shape.radius
            points.extend(self._sample_circle_boundary(center, radius))
        
        return points
    
    def _sample_polygon_boundary(self, vertices: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """폴리곤 경계에서 포인트 샘플링"""
        points = []
        
        for i in range(len(vertices)):
            start = vertices[i]
            end = vertices[(i + 1) % len(vertices)]
            
            # 두 점 사이의 거리
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            distance = np.sqrt(dx**2 + dy**2)
            
            # 필요한 포인트 수
            num_points = max(2, int(distance / self.resolution))
            
            for j in range(num_points):
                t = j / (num_points - 1) if num_points > 1 else 0
                x = start[0] + t * dx
                y = start[1] + t * dy
                points.append((x, y))
        
        return points
    
    def _sample_circle_boundary(self, center: Tuple[float, float], radius: float) -> List[Tuple[float, float]]:
        """원 경계에서 포인트 샘플링"""
        circumference = 2 * np.pi * radius
        num_points = max(8, int(circumference / self.resolution))
        
        points = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            points.append((x, y))
        
        return points
    
    def save_pointcloud(self, points: np.ndarray, filename: str, metadata: Optional[dict] = None) -> str:
        """
        포인트클라우드를 PLY 파일로 저장
        
        Args:
            points: (N, 2) numpy array
            filename: 파일명 (확장자 제외)
            metadata: 추가 메타데이터
            
        Returns:
            저장된 파일 경로
        """
        filepath = os.path.join(self.data_dir, f"{filename}.ply")
        self._save_ply(points, filepath)
        
        # 메타데이터 저장
        if metadata:
            meta_filepath = os.path.join(self.data_dir, f"{filename}_meta.json")
            with open(meta_filepath, 'w') as f:
                json.dump(metadata, f, indent=2)
        
        return filepath
    
    def _save_ply(self, points: np.ndarray, filepath: str):
        """PLY 형식으로 저장"""
        with open(filepath, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} 0.000000\n")
    
    def visualize_pointcloud(self, points: np.ndarray, title: str = "Pointcloud"):
        """포인트클라우드 시각화 (matplotlib 사용)"""
        try:
            import matplotlib.pyplot as plt
            
            plt.figure(figsize=(10, 8))
            plt.scatter(points[:, 0], points[:, 1], s=1, alpha=0.6)
            plt.title(title)
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.axis('equal')
            plt.grid(True, alpha=0.3)
            plt.show()
            
        except ImportError:
            print("matplotlib not available for visualization")
    
    def extract_and_save(self, world, workspace_bounds: Tuple[float, float, float, float], 
                        filename: str, format: str = 'npy') -> str:
        """환경에서 포인트클라우드를 추출하고 저장"""
        points = self.extract_from_world(world, workspace_bounds)
        
        metadata = {
            'num_points': len(points),
            'workspace_bounds': workspace_bounds,
            'resolution': self.resolution,
            'noise_level': self.noise_level,
            'format': format
        }
        
        filepath = self.save_pointcloud(points, filename, format, metadata)
        print(f"Pointcloud extracted: {len(points)} points saved to {filepath}")
        
        return filepath
