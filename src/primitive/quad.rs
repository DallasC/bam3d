//! Rectangular plane primitive

use glam::{Vec2, Vec3, Mat4};

use crate::{Aabb3, ray::Ray, volume::Sphere, traits::*};
use crate::primitive::util::get_max_point;

use crate::bound::{PlaneBound, Relation};
use crate::traits::*;
use crate::volume::{Aabb, MinMax};

/// Rectangular plane primitive. Will lie on the xy plane when not transformed.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Quad {
    /// Dimensions of the rectangle
    dim: Vec2,
    half_dim: Vec2,
    corners: [Vec3; 4],
}

impl Quad {
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: f32, dim_y: f32) -> Self {
        Self::new_impl(Vec2::new(dim_x, dim_y))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vec2) -> Self {
        let half_dim = dim / 2.;
        Quad {
            dim,
            half_dim,
            corners: Self::generate_corners(&half_dim),
        }
    }

    /// Get the dimensions of the `Rectangle`
    pub fn dim(&self) -> &Vec2 {
        &self.dim
    }

    /// Get the half dimensions of the `Rectangle`
    pub fn half_dim(&self) -> &Vec2 {
        &self.half_dim
    }

    fn generate_corners(half_dim: &Vec2) -> [Vec3; 4] {
        [
            Vec3::new(half_dim.x(), half_dim.y(), 0.),
            Vec3::new(-half_dim.x(), half_dim.y(), 0.),
            Vec3::new(-half_dim.x(), -half_dim.y(), 0.),
            Vec3::new(half_dim.x(), -half_dim.y(), 0.),
        ]
    }
}

impl Primitive for Quad {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        get_max_point(self.corners.iter(), direction, transform)
    }
}

impl ComputeBound<Aabb3> for Quad {
    fn compute_bound(&self) -> Aabb3 {
        Aabb3::new(
            Vec3::new(-self.half_dim.x(), -self.half_dim.y(), 0.),
            Vec3::new(self.half_dim.x(), self.half_dim.y(), 0.),
        )
    }
}

impl ComputeBound<Sphere> for Quad {
    fn compute_bound(&self) -> Sphere {
        Sphere {
            center: Vec3::zero(),
            radius: self.half_dim.x().max(self.half_dim.y()),
        }
    }
}

impl Discrete<Ray> for Quad {
    /// Ray must be in object space of the rectangle
    fn intersects(&self, ray: &Ray) -> bool {
        let aabb: Aabb3 = self.compute_bound();
        aabb.intersects(ray)
    }
}

impl Continuous<Ray> for Quad {
    type Result = Vec3;

    /// Ray must be in object space of the rectangle
    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        let aabb: Aabb3 = self.compute_bound();
        aabb.intersection(ray)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::algorithm::minkowski::GJK;
    use glam::{Vec3, Mat4, Quat};
    use crate::primitive::Cuboid;

    fn transform(x: f32, y: f32, z: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_x(1.0_f32.to_radians());
        let tran = Vec3::new(x, y, z);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }

    #[test]
    fn test_plane_cuboid_intersect() {
        let quad = Quad::new(2., 2.);
        let cuboid = Cuboid::new(1., 1., 1.);
        let transform_1 = transform(0., 0., 1.);
        let transform_2 = transform(0., 0., 1.1);
        let gjk = GJK::new();
        assert!(
            gjk.intersect(&quad, &transform_1, &cuboid, &transform_2)
                .is_some()
        );
    }
}