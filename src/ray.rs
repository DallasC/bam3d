//! Generic 3d rays
use glam::{Vec3, Mat4};
use crate::traits::{Continuous, ContinuousTransformed, Discrete, DiscreteTransformed};

/// A generic ray starting at `origin` and extending infinitely in
/// `direction`.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Ray {
    /// Ray origin
    pub origin: Vec3,
    /// Normalized ray direction
    pub direction: Vec3,
}

impl Ray {
    /// Create a generic ray starting at `origin` and extending infinitely in
    /// `direction`.
    pub fn new(origin: Vec3, direction: Vec3) -> Ray {
        Ray {
            origin,
            direction,
        }
    }

    /// Create a new ray by applying a transform.
    pub fn transform(self, transform: Mat4) -> Self {
        Self::new(
            transform.transform_point3(self.origin),
            transform.transform_vector3(self.direction),
        )
    }

}

impl Continuous<Ray> for Vec3 {
    type Result = Vec3;
    /// Find the intersection point. Returns None if nothing found
    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        if self.intersects(ray) {
            Some(*self)
        } else {
            None
        }
    }
}

impl Discrete<Ray> for Vec3 {
    /// Find if the ray intersects with a point
    fn intersects(&self, ray: &Ray) -> bool {
        let p = *self;
        let l = p - ray.origin;
        let tca = l.dot(ray.direction);
        tca > 0.
            && (((tca * tca) - l.dot(l)).abs() < std::f32::EPSILON)
    }
}

impl DiscreteTransformed<Ray> for dyn Discrete<Ray> {

    fn intersects_transformed(&self, ray: &Ray, transform: &Mat4) -> bool {
        self.intersects(&ray.transform(transform.inverse()))
    }
}

impl ContinuousTransformed<Ray> for dyn Continuous<Ray, Result = Vec3> {

    fn intersection_transformed(&self, ray: &Ray, transform: &Mat4) -> Option<Vec3> {

        self.intersection(&ray.transform(transform.inverse()))
            .map(|p| transform.transform_point3(p))
    }
}