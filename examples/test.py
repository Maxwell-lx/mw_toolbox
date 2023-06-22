import mw_toolbox.app as mw_core
import mw_toolbox.utils as mw_utils
import trimesh
import open3d as o3d

import cyfusion


meshfile = '../mw_toolbox/models/ArmadilloMesh.ply'

mesh = trimesh.load_mesh(meshfile)
views = cyfusion.PyViews(depthmaps, Ks,Rs,Ts)

# afterwards you can fuse the depth maps for example by
# depth,height,width: number of voxels in each dimension
# truncation: TSDF truncation value
tsdf = cyfusion.tsdf_gpu(views, depth,height,width, vx_size, truncation, False)

# the same code can also be run on the CPU
tsdf = cyfusion.tsdf_cpu(views, depth,height,width, vx_size, truncation, False, n_threads=8)