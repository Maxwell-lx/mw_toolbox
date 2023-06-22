import numpy as np
from .mw_math import homo2cart


def depth_to_pointcloud(depth_img, view_matrix, project_matrix, width, height):
    '''
    depth image(raw from opengl) to pointcloud
    :param depth_img: list, shape [width,height]
    :param view_matrix: list, shape [1,12], from opengl
    :param project_matrix: list, shape [1,12], from opengl
    :param width: width, int
    :param height: height, int
    :return: world_coords, pointcloud, shape [3,n]
    '''

    # 1.depth image to array
    x = np.linspace(0, width - 1, width)
    y = np.linspace(0, height - 1, height)
    x, y = np.meshgrid(x, y)
    z = depth_img.flatten()
    ones = np.ones_like(z)
    Xw = np.stack([x.flatten(), y.flatten(), z, ones])

    # 2. depth points to NDC points
    M_viewport = np.array([[width / 2, 0, 0, (width - 1) / 2], [0, -height / 2, 0, (height - 1) / 2], [0, 0, 1 / 2, 1 / 2], [0, 0, 0, 1]])
    NDC = np.linalg.inv(M_viewport) @ Xw

    # 3. delete background
    filter_mask = NDC[2, :] != 1  # 删除空白、无限远区域
    NDC = NDC[:, filter_mask]
    # 4. NDC to world_coords
    world_coords = np.linalg.inv(project_matrix @ view_matrix) @ NDC
    world_coords = homo2cart(world_coords).transpose([1, 0])
    return world_coords


def depth_to_distance(depth_img, far, near):
    '''
    depth image(raw from opengl) to pointcloud
    :param depth_img: list or ndarray, shape any
    :param far: far clip of opengl camera
    :param near: near clip of opengl camera
    :return: depth image with real depth value, unit:m
    '''
    return far * near / (far - (far - near) * depth_img)
