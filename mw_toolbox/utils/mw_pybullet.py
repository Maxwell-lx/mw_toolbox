import numpy as np


def draw_aabb(physics_client, aabb):
    '''
    draw aabb bounding box
    :param physics_client: pybullet id
    :param aabb: list, shape [2,3]
    :return: nothing
    '''
    aabbMin = aabb[0]
    aabbMax = aabb[1]
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMin[2]]
    physics_client.addUserDebugLine(f, t, [1, 0, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    physics_client.addUserDebugLine(f, t, [0, 1, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMin[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [0, 0, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    physics_client.addUserDebugLine(f, t, [1, 1, 1])


def draw_camera(physics_client, view_matrix, projection_matrix):
    '''
    draw OPENGL camera vison area
    :param physics_client: pybullet id
    :param view_matrix: list, shape [1,12]
    :param projection_matrix: list, shape [1,12]
    :return: nothing
    '''
    a = [-1, -1, -1, 1]
    b = [1, 1, 1, 1]
    c = [a[0], b[1], a[2], 1]
    d = [a[0], b[1], b[2], 1]
    e = [b[0], b[1], a[2], 1]
    f = [b[0], a[1], a[2], 1]
    g = [b[0], a[1], b[2], 1]
    h = [a[0], a[1], b[2], 1]
    index = np.array([a, b, c, d, e, f, g, h]).transpose([1, 0])

    edges = []
    for i in range(index.shape[1]):
        for j in range(i + 1, index.shape[1]):
            hamming_distance = np.sum(index[:, i] != index[:, j])
            if hamming_distance == 1:
                edges.append((i, j))

    points = np.linalg.inv(projection_matrix @ view_matrix) @ index
    points /= points[3, :]
    points = points[0:3, :]
    for edge in edges:
        f = [points[0][edge[0]], points[1][edge[0]], points[2][edge[0]]]
        t = [points[0][edge[1]], points[1][edge[1]], points[2][edge[1]]]
        physics_client.addUserDebugLine(f, t, [1, 0, 0])


def draw_sphere(physics_client, center, radius):
    '''
    draw an opacity sphere in pybullet client by given center and radius
    :param physics_client: pybullet id
    :param center: list, shape [1,3]
    :param radius: value
    :return: nothing
    '''
    sphere_visual_shape = physics_client.createVisualShape(
        shapeType=physics_client.GEOM_SPHERE,
        radius=radius,
        rgbaColor=(0, 0, 1, 0.3)  # 设置颜色为蓝色并设置透明度为0.3
    )

    # 创建一个没有质量和碰撞形状的多体对象
    bounding_sphere_id = physics_client.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=sphere_visual_shape,
        basePosition=center
    )

    return bounding_sphere_id



