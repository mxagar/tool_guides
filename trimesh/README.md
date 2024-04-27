# TriMesh User Guide

> [Trimesh](https://github.com/mikedh/trimesh) is a pure Python 3.7+ library for loading and using triangular meshes with an emphasis on watertight surfaces. The goal of the library is to provide a full featured and well tested Trimesh object which allows for easy manipulation and analysis, in the style of the Polygon object in the Shapely library.

The majority of examples in the section [TriMesh Guide](#trimesh-guide) and [`TriMesh.ipynb`](TriMesh.ipynb) come from the official site, as well as the used models.

Additionally, I created some tests in [`TriMesh_Tests.ipynb`](TriMesh_Tests.ipynb) to try some functionalities interesting for me personally.

Table of contents:
- [TriMesh User Guide](#trimesh-user-guide)
  - [Installation](#installation)
  - [TriMesh Guide](#trimesh-guide)
  - [Viewer](#viewer)
  - [TriMesh Tests](#trimesh-tests)
  - [Interesting Links](#interesting-links)

## Installation

```bash
# Standard installation
pip install trimesh

# Full installation, i.e., full experience with all the packages
pip install trimesh[all]
```

## TriMesh Guide

The notebook [`TriMesh.ipynb`](TriMesh.ipynb) has the following sections:

**1. Quickstart**
2. Section
3. Colors
4. Texture
5. Rays and Intersections
6. Nearest Points
7. Curvature Measurement

In the following, the contents of the first section are shown, which is probably the most useful and important ([source](https://trimesh.org/quick_start.html)).

```python
import numpy as np
import trimesh

MODELS_PATH = "../../trimesh/models"

# attach to logger so trimesh messages will be printed to console
#trimesh.util.attach_to_log()

# mesh objects can be created from existing faces and vertex data
mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 1, 0]],
                       faces=[[0, 1, 2]])

# by default, Trimesh will do a light processing, which will
# remove any NaN values and merge vertices that share position
# if you want to not do this on load, you can pass `process=False`
mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 1, 0]],
                       faces=[[0, 1, 2]],
                       process=False)

# load a file by name or from a buffer
mesh = trimesh.load_mesh(MODELS_PATH + '/featuretype.STL')
# to keep the raw data intact, disable any automatic processing
#mesh = trimesh.load_mesh(MODELS_PATH + '/featuretype.STL', process=False)

# is the current mesh watertight?
mesh.is_watertight

# what's the euler number for the mesh?
# https://en.wikipedia.org/wiki/Euler_characteristic
mesh.euler_number

# the convex hull is another Trimesh object that is available as a property
# lets compare the volume of our mesh with the volume of its convex hull
np.divide(mesh.volume, mesh.convex_hull.volume)

# since the mesh is watertight, it means there is a
# volumetric center of mass which we can set as the origin for our mesh
mesh.vertices -= mesh.center_mass

# what's the moment of inertia for the mesh?
mesh.moment_inertia

# if there are multiple bodies in the mesh we can split the mesh by
# connected components of face adjacency
# since this example mesh is a single watertight body we get a list of one mesh
mesh.split()

# preview mesh in a pyglet window from a terminal, or inline in a notebook
mesh.show()

# facets are groups of coplanar adjacent faces
# set each facet to a random color
# colors are 8 bit RGBA by default (n,4) np.uint8
for facet in mesh.facets:
    mesh.visual.face_colors[facet] = trimesh.visual.random_color()
mesh.show()

# a random rotation matrix: H[R|t] with t = 0
trimesh.transformations.random_rotation_matrix()

# transform method can be passed a (4,4) matrix and will cleanly apply the transform
mesh.apply_transform(trimesh.transformations.random_rotation_matrix())

# an axis aligned bounding box is available
mesh.bounding_box.primitive.extents

# a minimum volume oriented bounding box is available
mesh.bounding_box_oriented.primitive.extents

mesh.bounding_box_oriented.primitive.transform

# the bounding box is a trimesh.primitives.Box object, which subclasses
# Trimesh and lazily evaluates to fill in vertices and faces when requested
mesh.bounding_box_oriented.show()

# show the mesh appended with its oriented bounding box
# the bounding box is a trimesh.primitives.Box object, which subclasses
# Trimesh and lazily evaluates to fill in vertices and faces when requested
# (press w in viewer to see triangles)
(mesh + mesh.bounding_box_oriented).show()

# bounding spheres and bounding cylinders of meshes are also
# available, and will be the minimum volume version of each
# except in certain degenerate cases, where they will be no worse
# than a least squares fit version of the primitive.
print(mesh.bounding_box_oriented.volume,
      mesh.bounding_cylinder.volume,
      mesh.bounding_sphere.volume)
```

## Viewer

See [https://github.com/mikedh/trimesh/tree/main?tab=readme-ov-file#viewer](https://github.com/mikedh/trimesh/tree/main?tab=readme-ov-file#viewer).

> - mouse click + drag rotates the view
> - ctl + mouse click + drag pans the view
> - mouse wheel zooms
> - z returns to the base view
> - w toggles wireframe mode
> - c toggles backface culling
> - g toggles an XY grid with Z set to lowest point
> - a toggles an XYZ-RGB axis marker between: off, at world frame, or at every frame and world, and at every frame
> - f toggles between fullscreen and windowed mode
> - m maximizes the window
> - q closes the window

## TriMesh Tests

See [`TriMesh_Tests.ipynb`](TriMesh_Tests.ipynb).

## Interesting Links

- Common 3D models: [common-3d-test-models ](https://github.com/alecjacobson/common-3d-test-models)
- Source code and 3D models: [trimesh](https://github.com/mikedh/trimesh)
- [TriMesh Reference](https://trimesh.org/trimesh.html)
