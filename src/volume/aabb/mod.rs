//! Axis-aligned bounding boxes
//!
//! An AABB is a geometric object which encompasses a set of points and is not
//! rotated. It is either a rectangle or a rectangular prism (depending on the
//! dimension) where the slope of every line is either 0 or undefined. These
//! are useful for very cheap collision detection.

pub use self::aabb3::Aabb3;

use glam::{Vec3, Mat4};

use crate::traits::Bound;

mod aabb3;

/// Compute the minimum/maximum of the given values
pub trait MinMax {
    /// Compute the minimum
    fn min(a: Self, b: Self) -> Self;

    /// Compute the maximum
    fn max(a: Self, b: Self) -> Self;
}

impl MinMax for Vec3 {
    fn min(a: Vec3, b:Vec3) -> Vec3 {
        Vec3::new(a.x().min(b.x()), a.y().min(b.y()), a.z().min(b.z()))
    }

    fn max(a: Vec3, b: Vec3) -> Vec3 {
        Vec3::new(a.x().max(b.x()), a.y().max(b.y()), a.z().max(b.z()))
    }
}

/// Base trait describing an axis aligned bounding box.
pub trait Aabb: Sized {

    /// Create a new AABB using two points as opposing corners.
    fn new(p1: Vec3, p2: Vec3) -> Self;

    /// Create a new empty AABB
    fn zero() -> Self {
        let p = Vec3::zero();
        Aabb::new(p, p)
    }

    /// Return a shared reference to the point nearest to (-inf, -inf).
    fn min(&self) -> Vec3;

    /// Return a shared reference to the point nearest to (inf, inf).
    fn max(&self) -> Vec3;

    /// Return the dimensions of this AABB.
    #[inline]
    fn dim(&self) -> Vec3 {
        self.max() - self.min()
    }

    /// Return the volume this AABB encloses.
    #[inline]
    fn volume(&self) -> f32 {
        let dims = self.dim();
        dims.x() * dims.y() * dims.z()
    }

    /// Return the center point of this AABB.
    #[inline]
    fn center(&self) -> Vec3 {
        self.min() + self.dim() / 2.0
    }

    /// Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: Vec3) -> Self {
        Aabb::new(MinMax::min(self.min(), p), MinMax::max(self.max(), p))
    }

    /// Add a vector to every point in the AABB, returning a new AABB.
    #[inline]
    fn add_v(&self, v: Vec3) -> Self {
        Aabb::new(self.min() + v, self.max() + v)
    }

    /// Add a margin of the given width around the AABB, returning a new AABB.
    fn add_margin(&self, margin: Vec3) -> Self;

    /// Multiply every point in the AABB by a scalar, returning a new AABB.
    #[inline]
    fn mul_s(&self, s: f32) -> Self {
        Aabb::new(self.min() * s, self.max() * s)
    }

    /// Multiply every point in the AABB by a vector, returning a new AABB.
    fn mul_v(&self, v: Vec3) -> Self {
        let min = self.min() * v;
        let max = self.max() * v;
        Aabb::new(min, max)
    }

    /// Apply an arbitrary transform to the corners of this bounding box,
    /// return a new conservative bound.
    fn transform(&self, transform: &Mat4) -> Self;
    
}

impl<A> Bound for A
where
    A: Aabb,
{
    fn min_extent(&self) -> Vec3 {
        self.min()
    }

    fn max_extent(&self) -> Vec3 {
        self.max()
    }

    fn with_margin(&self, add: Vec3) -> Self {
        self.add_margin(add)
    }

    fn transform_volume(&self, transform: &Mat4) -> Self {
        self.transform(transform)
    }

    fn empty() -> Self {
        A::zero()
    }
}