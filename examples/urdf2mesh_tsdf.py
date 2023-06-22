import mw_toolbox.app as mw_app
import mw_toolbox.utils as mw_util
import trimesh
import open3d as o3d

file_path = "../mw_toolbox/models/urdf_box/mobility.urdf"
meshfile = 'extract.ply'
fixed_meshfile = 'extract_fixed.ply'
fix_method = 'MeshFix'  # 'MeshFix' or 'trimesh'
fix = True

# reconstruct mesh
mw_app.reconstruct_mesh_tsdf(file_path, output_meshfile=meshfile, voxel_length=0.01, sdf_trunc=0.04,num_views=800,resolution=1000)

# check watertight trimesh
mw_util.is_mesh_watertight(trimesh.load_mesh(meshfile))

# check watertight open3d
mesh = o3d.io.read_triangle_mesh(meshfile)
mw_util.check_properties(mesh)

# if not watertight, try to fix mesh
if fix:
    if fix_method == 'trimesh':
        mesh = trimesh.load_mesh(meshfile)
        mw_util.trimesh_fix_mesh(meshfile)
    elif fix_method == 'MeshFix':
        pass
        # mw_util.mesh_fix(meshfile)

    # check again trimesh
    mesh = trimesh.load_mesh(fixed_meshfile)
    mw_util.is_mesh_watertight(mesh)

    # check again open3d
    mesh = o3d.io.read_triangle_mesh(fixed_meshfile)
    mw_util.check_properties(mesh)
