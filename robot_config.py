"""
Robot Geometry Configurations
로봇 기하학적 구성 설정들
"""

# 기본 설정
DEFAULT_GEOMETRY = 0

# 로봇 구성 설정들
ROBOT_GEOMETRIES = {
    0: {
        "name": "Standard Rectangle Robot",
        "link_shape": "rectangle",
        "link_lengths": [3.0, 2.5, 2.0],
        "link_widths": [0.3, 0.25, 0.2],
        "description": "표준 직사각형 링크 로봇"
    },
    
    1: {
        "name": "Compact Rectangle Robot", 
        "link_shape": "rectangle",
        "link_lengths": [2.5, 2.0, 1.5],
        "link_widths": [0.25, 0.2, 0.15],
        "description": "소형 직사각형 링크 로봇"
    },
    
    2: {
        "name": "Extended Rectangle Robot",
        "link_shape": "rectangle", 
        "link_lengths": [3.5, 3.0, 2.5],
        "link_widths": [0.35, 0.3, 0.25],
        "description": "확장형 직사각형 링크 로봇"
    },
    
    3: {
        "name": "Standard Ellipse Robot",
        "link_shape": "ellipse",
        "link_lengths": [3.0, 2.5, 2.0],
        "link_widths": [0.3, 0.25, 0.2], 
        "description": "표준 타원형 링크 로봇"
    },
    
    4: {
        "name": "Slender Ellipse Robot",
        "link_shape": "ellipse",
        "link_lengths": [3.2, 2.8, 2.3],
        "link_widths": [0.2, 0.18, 0.15],
        "description": "슬렌더 타원형 링크 로봇"
    },
    
    5: {
        "name": "Heavy Duty Robot",
        "link_shape": "rectangle",
        "link_lengths": [4.0, 3.5, 3.0],
        "link_widths": [0.5, 0.45, 0.4],
        "description": "헤비듀티 산업용 로봇"
    }
}

def get_geometry_config(geometry_id: int) -> dict:
    """
    지정된 ID의 기하학적 구성을 반환
    
    Args:
        geometry_id: 기하학적 구성 ID
        
    Returns:
        geometry configuration dict
        
    Raises:
        ValueError: 유효하지 않은 geometry_id인 경우
    """
    if geometry_id not in ROBOT_GEOMETRIES:
        available_ids = list(ROBOT_GEOMETRIES.keys())
        raise ValueError(f"Invalid geometry_id: {geometry_id}. Available IDs: {available_ids}")
    
    return ROBOT_GEOMETRIES[geometry_id].copy()

def list_available_geometries() -> list:
    """사용 가능한 모든 기하학적 구성 목록 반환"""
    geometries = []
    for geo_id, config in ROBOT_GEOMETRIES.items():
        geometries.append({
            'id': geo_id,
            'name': config['name'],
            'shape': config['link_shape'], 
            'description': config['description']
        })
    return geometries

def print_available_geometries():
    """사용 가능한 기하학적 구성들을 출력"""
    print("Available Robot Geometries:")
    for geo_id, config in ROBOT_GEOMETRIES.items():
        shape = config['link_shape']
        name = config['name']
        desc = config['description']
        lengths = config['link_lengths']
        print(f"  {geo_id}: {name} ({shape}) - {desc}")
        print(f"      Lengths: {lengths}")
        print()
