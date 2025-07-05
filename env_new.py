# env.py
from Box2D.b2 import world, dynamicBody, staticBody, polygonShape, revoluteJointDef
import math

def create_ellipse_vertices(width, height, num_points=16):
    """타원형 버텍스 생성"""
    vertices = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = width * math.cos(angle)
        y = height * math.sin(angle)
        vertices.append((x, y))
    return vertices

def make_world(link_shape="rectangle"):
    W = world(gravity=(0, 0), doSleep=True)

    # 고정 베이스
    base = W.CreateStaticBody(position=(0,0))

    # 링크 설정
    link_lengths = [3.0, 2.5, 2.0]
    link_widths = [0.3, 0.25, 0.2]
    
    # 타원형 링크 설정
    ellipse_widths = [1.5, 1.25, 1.0]
    ellipse_heights = [0.15, 0.125, 0.1]
    
    links = []
    for i, length in enumerate(link_lengths):
        x = sum(link_lengths[:i]) + length/2
        body = W.CreateDynamicBody(position=(x, 0), angle=0)
        
        if link_shape == "ellipse":
            # 타원형 링크
            vertices = create_ellipse_vertices(ellipse_widths[i], ellipse_heights[i])
            body.CreatePolygonFixture(vertices=vertices, density=1, friction=0.3)
        else:
            # 기본 사각형 링크
            body.CreatePolygonFixture(box=(length/2, link_widths[i]/2), density=1, friction=0.3)
        
        links.append(body)

    # 관절 연결 (베이스-1, 1-2, 2-3)
    W.CreateJoint(revoluteJointDef(bodyA=base,   bodyB=links[0],
                                   localAnchorA=(0,0),
                                   localAnchorB=(-link_lengths[0]/2,0)))
    for i in range(len(links)-1):
        W.CreateJoint(revoluteJointDef(bodyA=links[i], bodyB=links[i+1],
                                       localAnchorA=(link_lengths[i]/2,0),
                                       localAnchorB=(-link_lengths[i+1]/2,0)))

    # 장애물 (static box)
    obstacles = []
    for pos in [(4,3), (6,1), (8,3)]:
        obs = W.CreateStaticBody(position=pos)
        obs.CreatePolygonFixture(box=(0.3, 0.3))
        obstacles.append(obs)

    return W, links, obstacles
