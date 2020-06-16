//! Utilities for use with
//! [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
//!

use std::marker::PhantomData;

use glam::Vec3;

use super::{ContinuousVisitor, DynamicBoundingVolumeTree, TreeValue, Visitor};
use crate::{ray::Ray, traits::*};

struct RayClosestVisitor<T>
where
    T: TreeValue,
{
    ray: Ray,
    min: f32,
    marker: PhantomData<T>,
}

impl<T> RayClosestVisitor<T>
where
    T: TreeValue,
{
    pub fn new(ray: Ray) -> Self {
        Self {
            ray,
            min: std::f32::INFINITY,
            marker: PhantomData,
        }
    }
}

impl<T> Visitor for RayClosestVisitor<T>
where
    T: TreeValue,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray, Result = Vec3>,
{
    type Bound = T::Bound;
    type Result = Vec3;

    fn accept(&mut self, bound: &Self::Bound, is_leaf: bool) -> Option<Self::Result> {
        match bound.intersection(&self.ray) {
            Some(point) => {
                let offset = point - self.ray.origin;
                let t = offset.dot(self.ray.direction);
                if t < self.min {
                    if is_leaf {
                        self.min = t;
                    }
                    Some(point)
                } else {
                    None
                }
            }
            None => None,
        }
    }
}

/// Query the given tree for the closest value that intersects the given ray.
///
/// ### Parameters:
///
/// - `tree`: DBVT to query.
/// - `ray`: Ray to find the closest intersection for.
///
/// ### Returns
///
/// Optionally returns the value that had the closest intersection with the ray, along with the
/// actual intersection point.
///
pub fn query_ray_closest<'a, T: 'a>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: Ray,
) -> Option<(&'a T, Vec3)>
where
    T: TreeValue,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray, Result = Vec3>
        + Discrete<Ray>,
{
    let mut saved = None;
    let mut tmin = std::f32::INFINITY;
    let mut visitor = RayClosestVisitor::<T>::new(ray);
    for (value, point) in tree.query(&mut visitor) {
        let offset = point - ray.origin;
        let t = offset.dot(ray.direction);
        if t < tmin {
            tmin = t;
            saved = Some((value, point));
        }
    }
    saved
}

/// Query the given tree for all values that intersects the given ray.
///
/// ### Parameters:
///
/// - `tree`: DBVT to query.
/// - `ray`: Ray to find intersections for.
///
/// ### Returns
///
/// Returns all values that intersected the ray, and also at which point the intersections occurred.
pub fn query_ray<'a, T: 'a>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: Ray,
) -> Vec<(&'a T, Vec3)>
where
    T: TreeValue,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray, Result = Vec3>
        + Discrete<Ray>,
{
    let mut visitor = ContinuousVisitor::<_, T>::new(&ray);
    tree.query(&mut visitor)
}