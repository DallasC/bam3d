use glam::{Vec3, Mat4};

use crate::{ray::Ray, Aabb3};
use crate::primitive::util::get_max_point;
use crate::volume::Sphere;

use crate::bound::{PlaneBound, Relation};
use crate::traits::*;
use crate::volume::{Aabb, MinMax};

/// Cuboid primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cuboid {
    /// Dimensions of the box
    dim: Vec3,
    half_dim: Vec3,
    corners: [Vec3; 8],
}

impl Cuboid {
    /// Create a new cuboid primitive from component dimensions
    pub fn new(dim_x: f32, dim_y: f32, dim_z: f32) -> Self {
        Self::new_impl(Vec3::new(dim_x, dim_y, dim_z))
    }

    /// Create a new cuboid primitive from a vector of component dimensions
    pub fn new_impl(dim: Vec3) -> Self {
        let half_dim = dim / 2.;
        Self {
            dim,
            half_dim,
            corners: Self::generate_corners(&half_dim),
        }
    }

    /// Get the dimensions of the `Cuboid`
    pub fn dim(&self) -> &Vec3 {
        &self.dim
    }

    /// Get the half dimensions of the `Cuboid`
    pub fn half_dim(&self) -> &Vec3 {
        &self.half_dim
    }

    fn generate_corners(half_dim: &Vec3) -> [Vec3; 8] {
        [
            Vec3::new(half_dim.x(), half_dim.y(), half_dim.z()),
            Vec3::new(-half_dim.x(), half_dim.y(), half_dim.z()),
            Vec3::new(-half_dim.x(), -half_dim.y(), half_dim.z()),
            Vec3::new(half_dim.x(), -half_dim.y(), half_dim.z()),
            Vec3::new(half_dim.x(), half_dim.y(), -half_dim.z()),
            Vec3::new(-half_dim.x(), half_dim.y(), -half_dim.z()),
            Vec3::new(-half_dim.x(), -half_dim.y(), -half_dim.z()),
            Vec3::new(half_dim.x(), -half_dim.y(), -half_dim.z()),
        ]
    }
}

impl Primitive for Cuboid {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        get_max_point(self.corners.iter(), direction, transform)
    }
}

impl ComputeBound<Aabb3> for Cuboid {
    fn compute_bound(&self) -> Aabb3 {
        Aabb3::new(
            -self.half_dim,
            self.half_dim,
        )
    }
}

impl ComputeBound<Sphere> for Cuboid {
    fn compute_bound(&self) -> Sphere {
        let max = self.half_dim.x().max(self.half_dim.y()).max(self.half_dim.z());
        Sphere {
            center: Vec3::zero(),
            radius: max,
        }
    }
}

impl Discrete<Ray> for Cuboid {
    fn intersects(&self, ray: &Ray) -> bool {
        Aabb3::new(
            -self.half_dim,
            self.half_dim,
        ).intersects(ray)
    }
}

impl Continuous<Ray> for Cuboid {
    type Result = Vec3;

    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        Aabb3::new(
            -self.half_dim,
            self.half_dim,
        ).intersection(ray)
    }
}

/// Cuboid primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cube {
    cuboid: Cuboid,
}

impl Cube {
    /// Create a new cube primitive
    pub fn new(dim: f32) -> Self {
        Cube {
            cuboid: Cuboid::new(dim, dim, dim),
        }
    }

    /// Get the dimension of the cube
    pub fn dim(&self) -> f32 {
        self.cuboid.dim.x()
    }

    /// Get the half dimension of the cube
    pub fn half_dim(&self) -> f32 {
        self.cuboid.half_dim.x()
    }
}

impl Primitive for Cube {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        self.cuboid.support_point(direction, transform)
    }
}

impl ComputeBound<Aabb3> for Cube {
    fn compute_bound(&self) -> Aabb3 {
        self.cuboid.compute_bound()
    }
}

impl ComputeBound<Sphere> for Cube {
    fn compute_bound(&self) -> Sphere {
        self.cuboid.compute_bound()
    }
}

impl Discrete<Ray> for Cube {
    fn intersects(&self, ray: &Ray) -> bool {
        self.cuboid.intersects(ray)
    }
}

impl Continuous<Ray> for Cube {
    type Result = Vec3;

    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        self.cuboid.intersection(ray)
    }
}

#[cfg(test)]
mod tests {

    use glam::{Vec3, Mat4, Quat};
    use crate::traits::*;

    use super::*;

    #[test]
    fn test_rectangle_bound() {
        let r = Cuboid::new(10., 10., 10.);
        assert_eq!(bound(-5., -5., -5., 5., 5., 5.), r.compute_bound())
    }

    #[test]
    fn test_ray_discrete() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        assert!(cuboid.intersects(&ray));
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(!cuboid.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.);
        assert!(cuboid.intersects_transformed(&ray, &t));
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(!cuboid.intersects_transformed(&ray, &t));
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.3);
        assert!(cuboid.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(Some(Vec3::new(5., 0., 0.)), cuboid.intersection(&ray));
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.);
        assert_eq!(
            Some(Vec3::new(5., 0., 0.)),
            cuboid.intersection_transformed(&ray, &t)
        );
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection_transformed(&ray, &t));
        let ray = Ray::new(Vec3::new(10., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.3);
        let p = cuboid.intersection_transformed(&ray, &t).unwrap();
        assert!((p.x() - 5.233_758).abs() < std::f32::EPSILON);
        assert!((p.y() - 0.).abs() < std::f32::EPSILON);
        assert!((p.z() - 0.).abs() < std::f32::EPSILON);
    }

    // util
    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_z(rot.to_radians());
        let tran = Vec3::new(dx, dy, dz);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }

    fn bound(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) -> Aabb3 {
        Aabb3::new(
            Vec3::new(min_x, min_y, min_z),
            Vec3::new(max_x, max_y, max_z),
        )
    }
}