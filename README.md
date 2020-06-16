# Bam3d
This library provides 3d collision detection primitives, bounding volumes and collision detection algorithms based on [glam](https://crates.io/crates/glam).

> This is currently a work in progress and not availble on crates.io yet. Once everything is working I'll upload it. 

The library provides:

- a generic ray: `Ray`
- a plane type: `Plane`
- a view frustum: `Frustum`
- axis-aligned bounding boxes: `Aabb2`, `Aabb3`
- oriented bounding boxes: `Obb2`, `Obb3`
- additional bounding volumes: `Sphere`, `Cylinder`
- collision primitives: `Sphere`, `Circle`, `Rectangle`, `Cuboid`, `Particle`, `Convex Polygon`, `Convex Polyhedra`
- a dynamic bounding volume tree (`DBVT`)
- broad phase collision detection: `Brute Force`, `Sweep and Prune`
- discrete narrow phase collision detection: `GJK` (including `EPA` for manifold computation)
- continuous narrow phase collision detection: `GJK`
- convex shape distance computation: `GJK`

## Cedits
This library is baed on the excellent [collision](https://crates.io/crates/collision) library. If you or a dependency already use [cgmath](https://crates.io/crates/cgmath) I recommend it.