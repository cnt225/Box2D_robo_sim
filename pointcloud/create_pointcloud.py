#!/usr/bin/env python3
"""
create_pointcloud.py - 포인트클라우드 생성 전용 스크립트
환경 설정과 포인트클라우드 추출을 담당
"""
import argparse
import sys
from pointcloud import PointcloudExtractor
from env import make_world


def parse_args():
    parser = argparse.ArgumentParser(description='Create pointcloud from static environment')
    
    # 출력 설정
    parser.add_argument('output_name', type=str,
                        help='Output pointcloud name (without .ply extension)')
    
    # 환경 설정
    parser.add_argument('--link-shape', choices=['rectangle', 'ellipse'], default='rectangle',
                        help='Link shape for environment generation (default: rectangle)')
    
    # 포인트클라우드 추출 설정
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Pointcloud resolution in meters (default: 0.05)')
    parser.add_argument('--noise-level', type=float, default=0.01,
                        help='Sensor noise level (default: 0.01)')
    parser.add_argument('--workspace-bounds', nargs=4, type=float, 
                        default=[-1, 11, -1, 11],
                        help='Workspace bounds: min_x max_x min_y max_y (default: -1 11 -1 11)')
    
    # 클러스터링 설정 (미리 메타데이터에 저장)
    parser.add_argument('--clustering-eps', type=float, default=0.3,
                        help='DBSCAN clustering epsilon for reconstruction (default: 0.3)')
    parser.add_argument('--min-samples', type=int, default=5,
                        help='DBSCAN minimum samples for reconstruction (default: 5)')
    parser.add_argument('--obstacle-type', choices=['polygon', 'circle', 'auto'], default='auto',
                        help='Obstacle type for reconstruction (default: auto)')
    
    # 시각화 옵션
    parser.add_argument('--visualize', action='store_true',
                        help='Show pointcloud visualization after extraction')
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    print(f"Creating pointcloud: {args.output_name}.ply")
    print(f"Link shape: {args.link_shape}")
    print(f"Resolution: {args.resolution}m")
    print(f"Noise level: {args.noise_level}")
    print(f"Workspace bounds: {args.workspace_bounds}")
    print()
    
    # 1. Static 환경 생성
    print("Generating static environment...")
    world, links, obstacles = make_world(args.link_shape, 'static')
    print(f"Created environment with {len(obstacles)} obstacles")
    
    # 2. 포인트클라우드 추출기 생성
    extractor = PointcloudExtractor(
        resolution=args.resolution, 
        noise_level=args.noise_level
    )
    
    # 3. 포인트클라우드 추출
    print("Extracting pointcloud...")
    workspace_bounds = tuple(args.workspace_bounds)
    points = extractor.extract_from_world(world, workspace_bounds)
    print(f"Extracted {len(points)} points")
    
    # 4. 메타데이터 준비 (클러스터링 설정 포함)
    metadata = {
        'link_shape': args.link_shape,
        'resolution': args.resolution,
        'noise_level': args.noise_level,
        'workspace_bounds': workspace_bounds,
        'clustering_eps': args.clustering_eps,
        'min_samples': args.min_samples,
        'obstacle_type': args.obstacle_type,
        'num_points': len(points),
        'num_obstacles': len(obstacles)
    }
    
    # 5. 파일 저장
    print("Saving pointcloud...")
    filepath = extractor.save_pointcloud(points, args.output_name, metadata=metadata)
    print(f"Saved: {filepath}")
    print(f"Metadata saved with clustering settings")
    
    # 6. 시각화 (옵션)
    if args.visualize:
        print("Showing visualization...")
        try:
            extractor.visualize_pointcloud(points, title=f"Pointcloud: {args.output_name}")
        except Exception as e:
            print(f"Visualization failed: {e}")
    
    print("\nPointcloud creation completed!")
    print(f"Use with: python main.py --env {args.output_name}.ply --target X Y")


if __name__ == "__main__":
    main()
