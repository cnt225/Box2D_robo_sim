# env.py
from Box2D.b2 import world, dynamicBody, staticBody, polygonShape, revoluteJointDef

def make_world():
    W = world(gravity=(0, 0), doSleep=True)

    # 고정 베이스
    base = W.CreateStaticBody(position=(0,0))

    # 링크 길이
    L = [3.0, 2.5, 2.0]
    links = []
    for i, length in enumerate(L):
        x = sum(L[:i]) + length/2
        body = W.CreateDynamicBody(position=(x, 0), angle=0)
        body.CreatePolygonFixture(box=(length/2, 0.1), density=1, friction=0.3)
        links.append(body)

    # 관절 연결 (베이스-1, 1-2, 2-3)
    W.CreateJoint(revoluteJointDef(bodyA=base,   bodyB=links[0],
                                   localAnchorA=(0,0),
                                   localAnchorB=(-L[0]/2,0)))
    for i in range(len(links)-1):
        W.CreateJoint(revoluteJointDef(bodyA=links[i], bodyB=links[i+1],
                                       localAnchorA=(L[i]/2,0),
                                       localAnchorB=(-L[i+1]/2,0)))

    # 장애물 (static box)
    obstacles = []
    for pos in [(4,3), (6,1), (8,3)]:
        obs = W.CreateStaticBody(position=pos)
        obs.CreatePolygonFixture(box=(0.3, 0.3))
        obstacles.append(obs)

    return W, links, obstacles