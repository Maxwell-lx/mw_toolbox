import numpy as np
import trimesh


def is_mesh_watertight(mesh, print_info=True):
    '''
    在一个正确的mesh文件中，一个边不会被使用三次，因此只要邻接边数量与边的总数量之差不为0，或孤立顶点数量>0, 则模型不是watertight
    但是反之，如果上述条件，满足，则模型不一定是watertight，面封闭 不等于 面不交叉，如果存在面交叉的情况，就会造成局部无法定义在模型内或模型外，模型不是watertight
    :param mesh: trimesh or mesh file path
    :return: bool, is watertight or not
    '''
    # count isolated vertices
    used_vertices = np.unique(mesh.faces.flatten())
    isolated_vertices_count = len(mesh.vertices) - len(used_vertices)

    # count isolated edges
    edges_unique = len(mesh.edges_unique)
    face_adjacency_edges = len(mesh.face_adjacency_edges)
    isolated_edges_count = edges_unique - face_adjacency_edges

    is_watertight = mesh.is_watertight
    mesh.is_mesh
    if print_info:
        print('Number of isolated vertices in the mesh:', isolated_vertices_count)
        print('Number of edges used only once:', isolated_edges_count)
        print('is watertight by isolated vertices and edges count:', not (isolated_vertices_count > 0 or isolated_edges_count > 0))
        print('Is mesh watertight:', is_watertight)
    return is_watertight


def trimesh_fix_mesh(meshfile, outfile='default'):
    '''
    try to fix mesh through trimesh's function
    :param mesh: mesh
    :return: nothing
    '''
    mesh = trimesh.load_mesh(meshfile)
    # remove isolated vertices
    mesh.remove_unreferenced_vertices()
    # fill hole
    mesh.fill_holes()
    if outfile == 'default':
        outfile = meshfile.split('.')[0] + '_fixed.stl'
    mesh.export(outfile)
