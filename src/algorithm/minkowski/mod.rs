//! Algorithms using the Minkowski Sum/Difference

pub use self::epa::{EPA3, EPA};
pub use self::gjk::{SimplexProcessor, GJK};

use std::ops::{Neg, Sub};
use glam::{Vec3, Mat4};

use crate::traits::*;

mod epa;
mod gjk;

/// Minkowski Sum/Difference support point
#[derive(Clone, Debug, Copy)]
pub struct SupportPoint {
    v: Vec3,
    sup_a: Vec3,
    sup_b: Vec3,
}

impl SupportPoint {
    /// Create a new support point at origin
    pub fn new() -> Self {
        Self {
            v: Vec3::zero(),
            sup_a: Vec3::zero(),
            sup_b: Vec3::zero(),
        }
    }

    /// Create a new support point from the minkowski difference, using primitive support functions
    pub fn from_minkowski<SL, SR>(
        left: &SL,
        left_transform: &Mat4,
        right: &SR,
        right_transform: &Mat4,
        direction: &Vec3,
    ) -> Self
    where
        SL: Primitive,
        SR: Primitive,
    {
        let l = left.support_point(direction, left_transform);
        let r = right.support_point(&direction.neg(), right_transform);
        Self {
            v: l - r,
            sup_a: l,
            sup_b: r,
        }
    }
}

impl Default for SupportPoint {
    fn default() -> Self {
        SupportPoint::new()
    }
}

impl Sub<Vec3> for SupportPoint {
    type Output = Self;

    fn sub(self, rhs: Vec3) -> Self {
        SupportPoint {
            v: self.v - rhs,
            sup_a: self.sup_a,
            sup_b: self.sup_b,
        }
    }
}

//TODO 3d test