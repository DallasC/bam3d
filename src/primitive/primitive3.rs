//! Wrapper enum for 3D primitives

use glam::{Vec3, Mat4};

use crate::{Aabb3, ray::Ray};
use crate::primitive::{Capsule, ConvexPolyhedron, Cube, Cuboid, Cylinder, Particle3, Quad, Sphere};

pub use crate::bound::{PlaneBound, Relation};
pub use crate::traits::*;
pub use crate::volume::{Aabb, MinMax};

/// Wrapper enum for 3D primitives, that also implements the `Primitive` trait, making it easier
/// to use many different primitives in algorithms.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Primitive3 {
    /// Particle
    Particle(Particle3),
    /// Rectangular plane
    Quad(Quad),
    /// Sphere
    Sphere(Sphere),
    /// Cuboid
    Cuboid(Cuboid),
    /// Cube
    Cube(Cube),
    /// Cylinder
    Cylinder(Cylinder),
    /// Capsule
    Capsule(Capsule),
    /// Convex polyhedron with any number of vertices/faces
    ConvexPolyhedron(ConvexPolyhedron),
}

impl From<Particle3> for Primitive3 {
    fn from(particle: Particle3) -> Primitive3 {
        Primitive3::Particle(particle)
    }
}

impl From<Quad> for Primitive3 {
    fn from(quad: Quad) -> Self {
        Primitive3::Quad(quad)
    }
}

impl From<Sphere> for Primitive3 {
    fn from(sphere: Sphere) -> Primitive3 {
        Primitive3::Sphere(sphere)
    }
}

impl From<Cube> for Primitive3 {
    fn from(cuboid: Cube) -> Primitive3 {
        Primitive3::Cube(cuboid)
    }
}

impl From<Cuboid> for Primitive3 {
    fn from(cuboid: Cuboid) -> Primitive3 {
        Primitive3::Cuboid(cuboid)
    }
}

impl From<Cylinder> for Primitive3 {
    fn from(cylinder: Cylinder) -> Primitive3 {
        Primitive3::Cylinder(cylinder)
    }
}

impl From<Capsule> for Primitive3 {
    fn from(capsule: Capsule) -> Primitive3 {
        Primitive3::Capsule(capsule)
    }
}

impl From<ConvexPolyhedron> for Primitive3 {
    fn from(polyhedron: ConvexPolyhedron) -> Primitive3 {
        Primitive3::ConvexPolyhedron(polyhedron)
    }
}

impl ComputeBound<Aabb3> for Primitive3 {
    fn compute_bound(&self) -> Aabb3 {
        match *self {
            Primitive3::Particle(_) => Aabb3::zero(),
            Primitive3::Quad(ref quad) => quad.compute_bound(),
            Primitive3::Cuboid(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Cube(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Sphere(ref sphere) => sphere.compute_bound(),
            Primitive3::Cylinder(ref cylinder) => cylinder.compute_bound(),
            Primitive3::Capsule(ref capsule) => capsule.compute_bound(),
            Primitive3::ConvexPolyhedron(ref polyhedron) => polyhedron.compute_bound(),
        }
    }
}

impl ComputeBound<crate::volume::Sphere> for Primitive3 {
    fn compute_bound(&self) -> crate::volume::Sphere {
        match *self {
            Primitive3::Particle(_) => crate::volume::Sphere {
                center: Vec3::zero(),
                radius: 0.,
            },
            Primitive3::Quad(ref quad) => quad.compute_bound(),
            Primitive3::Cuboid(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Cube(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Sphere(ref sphere) => sphere.compute_bound(),
            Primitive3::Cylinder(ref cylinder) => cylinder.compute_bound(),
            Primitive3::Capsule(ref capsule) => capsule.compute_bound(),
            Primitive3::ConvexPolyhedron(ref polyhedron) => polyhedron.compute_bound(),
        }
    }
}

impl Primitive for Primitive3 {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        match *self {
            Primitive3::Particle(_) => transform.transform_point3(Vec3::zero()),
            Primitive3::Quad(ref quad) => quad.support_point(direction, transform),
            Primitive3::Sphere(ref sphere) => sphere.support_point(direction, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.support_point(direction, transform),
            Primitive3::Cube(ref cuboid) => cuboid.support_point(direction, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.support_point(direction, transform),
            Primitive3::Capsule(ref capsule) => capsule.support_point(direction, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.support_point(direction, transform)
            }
        }
    }
}

impl DiscreteTransformed<Ray> for Primitive3 {
    fn intersects_transformed(&self, ray: &Ray, transform: &Mat4) -> bool {
        match *self {
            Primitive3::Particle(ref particle) => particle.intersects_transformed(ray, transform),
            Primitive3::Quad(ref quad) => quad.intersects_transformed(ray, transform),
            Primitive3::Sphere(ref sphere) => sphere.intersects_transformed(ray, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.intersects_transformed(ray, transform),
            Primitive3::Cube(ref cuboid) => cuboid.intersects_transformed(ray, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.intersects_transformed(ray, transform),
            Primitive3::Capsule(ref capsule) => capsule.intersects_transformed(ray, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.intersects_transformed(ray, transform)
            }
        }
    }
}

impl ContinuousTransformed<Ray> for Primitive3 {
    fn intersection_transformed(&self, ray: &Ray, transform: &Mat4) -> Option<Vec3> {
        match *self {
            Primitive3::Particle(ref particle) => particle.intersection_transformed(ray, transform),
            Primitive3::Quad(ref quad) => quad.intersection_transformed(ray, transform),
            Primitive3::Sphere(ref sphere) => sphere.intersection_transformed(ray, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.intersection_transformed(ray, transform),
            Primitive3::Cube(ref cuboid) => cuboid.intersection_transformed(ray, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.intersection_transformed(ray, transform),
            Primitive3::Capsule(ref capsule) => capsule.intersection_transformed(ray, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.intersection_transformed(ray, transform)
            }
        }
    }
}