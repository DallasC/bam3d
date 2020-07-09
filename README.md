# Bam3d
This library provides 3d collision detection primitives, bounding volumes and collision detection algorithms using [glam](https://crates.io/crates/glam).

> If you need 2d collision detection check out [bam2d](https://github.com/dallasc/bam2d)

## Features

The library provides:

- a generic ray: `Ray`
- a plane type: `Plane`
- a view frustum: `Frustum`
- axis-aligned bounding boxes: `Aabb3`
- oriented bounding boxes: `Obb3`
- additional bounding volumes: `Sphere`, `Cylinder`
- collision primitives: `Sphere`, `Circle`, `Rectangle`, `Cuboid`, `Particle`, `Convex Polygon`, `Convex Polyhedra`
- a dynamic bounding volume tree (`DBVT`)
- broad phase collision detection: `Sweep and Prune`
- discrete narrow phase collision detection: `GJK` (including `EPA` for manifold computation)
- continuous narrow phase collision detection: `GJK`
- convex shape distance computation: `GJK`

## Examples
Right now the docs are probably are all there is. I'll hopefully be able to add some examples soon<sup>tm</sup>.

## Cedits
This started as a port of the excellent [cgmath](https://crates.io/crates/cgmath) based [collision](https://crates.io/crates/collision) library. If you or your dependency tree already use [cgmath](https://crates.io/crates/cgmath) then it could be an excellent option.