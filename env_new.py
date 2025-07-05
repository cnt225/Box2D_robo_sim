# env.py - Environment Creation Module (모듈 2: 시뮬레이션 실행)
from Box2D.b2 import world, dynamicBody, staticBody, polygonShape, revoluteJointDef
import math
import os
import numpy as np
from typing import List, Tuple, Optional
from pointcloud import PointcloudLoader
from robot_config import get_geometry_config


def create_ellipse_vertices(width, height, num_points=16):
    """타원형 버텍스 생성"""
    vertices = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = width * math.cos(angle)
        y = height * math.sin(angle)
        vertices.append((x, y))
    return vertices


def make_world(geometry_id=0, env_file=None):
    """
    Create world with robot and environment
    
    Args:
        geometry_id: Robot geometry configuration ID (0-5)
        env_file: Pointcloud environment file (.ply). If None, uses static environment
        
    Returns:
        tuple: (world, links, obstacles)
    """
    # Get robot geometry configuration
    try:
        geometry_config = get_geometry_config(geometry_id)
        print(f"Using geometry {geometry_id}: {geometry_config['name']}")
    except ValueError as e:
        print(f"Error: {e}")
        print("Using default geometry (ID: 0)")
        geometry_config = get_geometry_config(0)
    
    if env_file:
        return _make_pointcloud_world(geometry_config, env_file)
    else:
        return _make_static_world(geometry_config)


def _make_static_world(geometry_config):
    """Create static world with predefined obstacles"""
    W = world(gravity=(0, 0), doSleep=True)

    # Create robot using geometry config
    links = _create_robot_links(W, geometry_config)

    # 장애물 생성 (기존 정적 장애물들)
    obstacles = []

    # 장애물 1: 큰 사각형 
    obs1 = W.CreateStaticBody(
        position=(6, 2),
        shapes=polygonShape(box=(1.5, 0.5))
    )
    obstacles.append(obs1)

    # 장애물 2: 작은 사각형
    obs2 = W.CreateStaticBody(
        position=(4, 1),
        shapes=polygonShape(box=(0.5, 0.5))
    )
    obstacles.append(obs2)

    # 장애물 3: 세로 긴 사각형
    obs3 = W.CreateStaticBody(
        position=(8, 1.5),
        shapes=polygonShape(box=(0.3, 1.0))
    )
    obstacles.append(obs3)

    return W, links, obstacles


def _make_pointcloud_world(geometry_config, env_file):
    """Create world from pointcloud file (uses metadata for all settings)"""
    loader = PointcloudLoader()
    
    try:
        W = loader.load_and_create_world(env_file)
        print(f"Loaded pointcloud environment from {env_file}")
        
    except Exception as e:
        print(f"Failed to load pointcloud environment: {e}")
        print("Falling back to empty environment")
        W = world(gravity=(0, 0), doSleep=True)
    
    # Create robot in the pointcloud world
    links = _create_robot_links(W, geometry_config)
    
    # Extract obstacles from world (all static bodies except robot base)
    # Robot base is the first static body created
    obstacles = []
    robot_base = None
    for body in W.bodies:
        if body.type == staticBody:
            if robot_base is None:
                robot_base = body  # First static body is the robot base
            else:
                obstacles.append(body)  # Others are obstacles
    
    return W, links, obstacles


def _create_robot_links(W, geometry_config):
    """Create robot links in given world using geometry configuration"""
    # Get geometry parameters
    link_shape = geometry_config['link_shape']
    link_lengths = geometry_config['link_lengths']
    link_widths = geometry_config['link_widths']
    
    # 고정 베이스
    base = W.CreateStaticBody(position=(0, 0))
    
    # 링크 생성
    links = []
    prev_body = base
    current_pos = (0, 0)

    for i, (length, width) in enumerate(zip(link_lengths, link_widths)):
        # 링크 생성
        if link_shape == "ellipse":
            # 타원형 링크
            ellipse_vertices = create_ellipse_vertices(width/2, length/2)
            link = W.CreateDynamicBody(
                position=(current_pos[0], current_pos[1] + length/2),
                shapes=polygonShape(vertices=ellipse_vertices)
            )
        else:
            # 직사각형 링크 (기본)
            link = W.CreateDynamicBody(
                position=(current_pos[0], current_pos[1] + length/2),
                shapes=polygonShape(box=(width/2, length/2))
            )

        # 조인트 생성
        joint_def = revoluteJointDef(
            bodyA=prev_body,
            bodyB=link,
            anchor=(current_pos[0], current_pos[1]),  # 월드 좌표
            enableLimit=True,
            lowerAngle=-math.pi,
            upperAngle=math.pi
        )
        W.CreateJoint(joint_def)

        links.append(link)
        prev_body = link
        current_pos = (current_pos[0], current_pos[1] + length)

    return links


# 편의 함수들 (하위 호환성)
def list_available_pointclouds():
    """List available pointcloud files"""
    loader = PointcloudLoader()
    return loader.list_available_pointclouds()
