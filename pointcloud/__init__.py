"""
Pointcloud module for Box2D robot simulation.

This module provides functionality to:
1. Extract point clouds from Box2D environments
2. Load point clouds and convert them back to Box2D environments
3. Support various file formats (npy, ply, txt)

Usage:
    from pointcloud import PointcloudExtractor, PointcloudLoader
    
    # Extract point cloud from environment
    extractor = PointcloudExtractor()
    points = extractor.extract_from_world(world, bounds)
    extractor.save_pointcloud(points, "my_env", format="npy")
    
    # Load point cloud and create environment
    loader = PointcloudLoader()
    new_world = loader.load_and_create_world("my_env", format="npy")
"""

from .pointcloud_extractor import PointcloudExtractor
from .pointcloud_loader import PointcloudLoader

__all__ = ['PointcloudExtractor', 'PointcloudLoader']
