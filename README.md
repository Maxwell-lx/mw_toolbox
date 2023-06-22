# mw_toolbox
This package contains various useful functions for 3D deep learning.

all code are tested on linux

# install 


To fully install this package, run

```bash
cd mw_toolbox
chmod +x install.sh
./install.sh
```

# examples

urdf2mesh_tsdf

check_mesh_property

> watertight mesh:
>
> "Edge Manifold": In an edge manifold mesh, each edge is shared by only two triangles. This means that no separate edge is connected to more than two faces; each edge is either on the edge of the mesh or completely surrounded by two faces.
>
> if allow_boundary_edges then each edge should be shared by 1 or 2 triangles
>
> if not allow_boundary_edges then each edge should be shared by 2 triangles
>
> "Vertex Manifold": In a vertex manifold mesh, all the faces surrounding a vertex form a continuous "ring" on the surface of the mesh, with no gaps. This means that the faces adjacent to each vertex are visually continuous, with no breaks.
>
> 
>
> "Not Self-Intersecting": This means that any two faces of the mesh do not intersect each other. That is, if you observe the mesh from the inside or outside, you will not see any face passing through another face.





## Reference

* [QhelDIV / xgutils](https://github.com/QhelDIV/xgutils)
* [marian42/mesh_to_sdf: Calculate signed distance fields for arbitrary meshes (github.com)](https://github.com/marian42/mesh_to_sdf)
