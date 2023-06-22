import pybullet as p
import numpy as np
import open3d as o3d

from ..utils import mw_image
from ..utils import mw_math
from ..utils import mw_pybullet as pb
from ..utils import mw_pybullet_class as pbc

from tqdm import tqdm
import matplotlib.pyplot as plt
import math


def reconstruct_mesh_tsdf(file_path, output_meshfile='output_meshfile.stl', debug=False, voxel_length=0.005, sdf_trunc=0.03, num_views=50, resolution=500):
    aspect = 1
    height = resolution
    width = aspect * height

    fov = 60
    fov_r = np.pi * fov / 180

    # near = 0.1
    # far = 4

    # init PyBullet
    if debug:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # load URDF
    robot_id = p.loadURDF(file_path, [0, 0, 0])
    robot = pbc.pybullet_robot(p, robot_id)
    if debug:
        robot.print()  # print robot info

    # get aabbs
    aabbs = []
    for i in range(robot.joint_num):
        aabbs.append(p.getAABB(robot.id, i))
        if debug:
            pb.draw_aabb(p, aabbs[i])

    # get bounding sphere
    center, radius = mw_math.get_aabbs_bounding_sphere(aabbs)
    if debug and False:
        pb.draw_sphere(p, center, radius)

    near = radius
    far = radius * 3

    # sampling camera position on fibonacci sphere
    points = mw_math.fibonacci_sphere_sample(center, radius * 2, num_views)
    normals = points - center  # inline with camera's z-axis
    random_vectors = np.random.randn(*normals.shape)
    cam_ups = np.cross(normals, random_vectors)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    # intrinsics
    intrinsics = o3d.camera.PinholeCameraIntrinsic()
    f_x = f_y = height / (2 * math.tan(fov_r / 2))
    intrinsics.set_intrinsics(width, height, f_x, f_y, width / 2 - 0.5, height / 2 - 0.5)

    M_view = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    for i in tqdm(range(num_views), desc=file_path):
        position = points[i]
        target = center
        camup = cam_ups[i]

        # get camera parameters
        view_matrix = p.computeViewMatrix(position, target, camup)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        _, _, rgb_img, depth_img, _ = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # reshape list to matrix
        view_matrix = np.array(p.computeViewMatrix(position, target, camup)).reshape((4, 4), order='F')
        projection_matrix = np.array(p.computeProjectionMatrixFOV(fov, aspect, near, far)).reshape((4, 4), order='F')

        # draw camera vision area
        if debug:
            pb.draw_camera(p, view_matrix, projection_matrix)
            world_points = mw_image.depth_to_pointcloud(depth_img, view_matrix, projection_matrix, width, height)
            color_list = [[1, 0, 0] for _ in range(len(world_points))]
            p.addUserDebugPoints(world_points, color_list, 1)
            plt.imshow(rgb_img)
            plt.colorbar()
            plt.title("depth image")
            plt.show(block=False)
            plt.pause(2)
            plt.close()

        rgb_img_3d = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 3), dtype=np.uint8)
        rgb_img_3d[:, :, 0] = rgb_img[:, :, 0]
        rgb_img_3d[:, :, 1] = rgb_img[:, :, 1]
        rgb_img_3d[:, :, 2] = rgb_img[:, :, 2]
        rgb_o3d = o3d.geometry.Image(rgb_img_3d)

        depth_img = depth_img.astype(np.float32)
        depth_img = mw_image.depth_to_distance(depth_img, far, near)
        depth_o3d = o3d.geometry.Image(depth_img)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d, 1, 0.95 * far, False)
        if debug:
            o3d.visualization.draw_geometries([rgbd_image])

        volume.integrate(rgbd_image, intrinsics, M_view @ view_matrix)

    p.disconnect()

    # extract mesh data TSDF
    print('extracting mesh ...')
    mesh = volume.extract_triangle_mesh()
    print('compute vertex normals ...')
    mesh.compute_vertex_normals()
    # print('visualization ...')
    # o3d.visualization.draw_geometries([mesh])

    # save mesh
    o3d.io.write_triangle_mesh(output_meshfile, mesh)
    print(output_meshfile,' saved.')