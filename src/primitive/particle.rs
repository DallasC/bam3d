//! Particle primitive

use std::marker;
use std::ops::Range;

use glam::{Vec3, Mat4};

use crate::{ray::Ray, traits::*};

/// Represents a particle in space.
///
/// Only have implementations of intersection tests for movement,
/// using `(Particle, Range<P>)` where `P` is the positional `Point` of the particle at the start
/// and end of movement.
/// These intersection tests can be used with any primitive that has `Ray` intersection tests
/// implemented.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Particle<P> {
    m: marker::PhantomData<P>,
}

impl<P> Particle<P> {
    /// Create a new particle
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}
/// 3D particle
pub type Particle3 = Particle<Vec3>;

impl<C> Discrete<(Particle<Vec3>, Range<Vec3>)> for C
where
    C: Continuous<Ray, Result = Vec3>,
{
    fn intersects(&self, &(_, ref range): &(Particle<Vec3>, Range<Vec3>)) -> bool {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        match self.intersection(&ray) {
            None => false,
            Some(p) => (p - range.start).dot(p - range.start) <= direction.dot(direction),
        }
    }
}

impl Discrete<Ray> for Particle<Vec3> {
    /// Ray needs to be in particle object space
    fn intersects(&self, ray: &Ray) -> bool {
        Vec3::zero().intersects(ray)
    }
}

impl Continuous<Ray> for Particle<Vec3> {
    type Result = Vec3;

    /// Ray needs to be in particle object space
    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        Vec3::zero().intersection(ray)
    }
}



impl<C> DiscreteTransformed<(Particle<Vec3>, Range<Vec3>)> for C 
where
    C: ContinuousTransformed<Ray>
{
    fn intersects_transformed(
        &self,
        &(_, ref range): &(Particle<Vec3>, Range<Vec3>),
        transform: &Mat4,
    ) -> bool {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        match self.intersection_transformed(&ray, transform) {
            None => false,
            Some(p) => (p - range.start).dot(p - range.start) <= direction.dot(direction),
        }
    }
}

impl<C> Continuous<(Particle<Vec3>, Range<Vec3>)> for C
where
    C: Continuous<Ray, Result = Vec3> 
{
    type Result = Vec3;

    fn intersection(&self, &(_, ref range): &(Particle<Vec3>, Range<Vec3>)) -> Option<Vec3> {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        self.intersection(&ray).and_then(|p| {
            if (p - range.start).dot(p - range.start) <= direction.dot(direction) {
                Some(p)
            } else {
                None
            }
        })
    }
}

impl<C> ContinuousTransformed<(Particle<Vec3>, Range<Vec3>)> for C 
where
    C: ContinuousTransformed<Ray> 
{
    fn intersection_transformed(
        &self,
        &(_, ref range): &(Particle<Vec3>, Range<Vec3>),
        transform: &Mat4,
    ) -> Option<Vec3> {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        self.intersection_transformed(&ray, transform)
            .and_then(|p| {
                if (p - range.start).dot(p - range.start) <= direction.dot(direction) {
                    Some(p)
                } else {
                    None
                }
            })
    }
}

// TODO: Add particle tests for 3d