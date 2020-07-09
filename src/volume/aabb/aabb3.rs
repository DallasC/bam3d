//! Axis aligned bounding box for 3D.
//!

use std::fmt;

use glam::{Vec3, Mat4};

use crate::{line::Line, plane::Plane, ray::Ray, volume::Sphere, traits::*};

use crate::bound::{PlaneBound, Relation};
use crate::volume::{Aabb};

/// A three-dimensional AABB, aka a rectangular prism.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Aabb3 {
    /// Minimum point of the AABB
    pub min: Vec3,
    /// Maximum point of the AABB
    pub max: Vec3,
}

impl Aabb3 {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Vec3, p2: Vec3) -> Aabb3 {
        Aabb3 {
            min: Vec3::new(p1.x().min(p2.x()), p1.y().min(p2.y()), p1.z().min(p2.z())),
            max: Vec3::new(p1.x().max(p2.x()), p1.y().max(p2.y()), p1.z().max(p2.z())),
        }
    }

    /// Compute corners.
    #[inline]
    pub fn to_corners(&self) -> [Vec3; 8] {
        [
            self.min,
            Vec3::new(self.max.x(), self.min.y(), self.min.z()),
            Vec3::new(self.min.x(), self.max.y(), self.min.z()),
            Vec3::new(self.max.x(), self.max.y(), self.min.z()),
            Vec3::new(self.min.x(), self.min.y(), self.max.z()),
            Vec3::new(self.max.x(), self.min.y(), self.max.z()),
            Vec3::new(self.min.x(), self.max.y(), self.max.z()),
            self.max,
        ]
    }
}

impl Aabb for Aabb3 {

    #[inline]
    fn new(p1: Vec3, p2: Vec3) -> Aabb3 {
        Aabb3::new(p1, p2)
    }

    #[inline]
    fn zero() -> Aabb3 {
        let p = Vec3::zero();
        Aabb3::new(p, p)
    }

    #[inline]
    fn min(&self) -> Vec3 {
        self.min
    }

    #[inline]
    fn max(&self) -> Vec3 {
        self.max
    }

    #[inline]
    fn add_margin(&self, margin: Vec3) -> Self {
        Aabb3::new(
            Vec3::new(
                self.min.x() - margin.x(),
                self.min.y() - margin.y(),
                self.min.z() - margin.z(),
            ),
            Vec3::new(
                self.max.x() + margin.x(),
                self.max.y() + margin.y(),
                self.max.z() + margin.z(),
            ),
        )
    }
    
    #[inline]
    fn transform(&self, transform: &Mat4) -> Self {
        let corners = self.to_corners();
        let transformed_first = transform.transform_point3(corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..]
            .iter()
            .fold(base, |u, &corner| u.grow(transform.transform_point3(corner)))
    }
}

impl fmt::Debug for Aabb3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}

impl Contains<Vec3> for Aabb3 {
    #[inline]
    fn contains(&self, p: &Vec3) -> bool {
        self.min.x() <= p.x() && p.x() < self.max.x() && self.min.y() <= p.y() && p.y() < self.max.y()
            && self.min.z() <= p.z() && p.z() < self.max.z()
    }
}

impl Contains<Aabb3> for Aabb3 {
    #[inline]
    fn contains(&self, other: &Aabb3) -> bool {
        let other_min = other.min();
        let other_max = other.max();

        other_min.x() >= self.min.x() && other_min.y() >= self.min.y() && other_min.z() >= self.min.z()
            && other_max.x() <= self.max.x() && other_max.y() <= self.max.y()
            && other_max.z() <= self.max.z()
    }
}

impl Contains<Sphere> for Aabb3 {
    // will return true for border hits on both min and max extents
    #[inline]
    fn contains(&self, sphere: &Sphere) -> bool {
        (sphere.center.x() - sphere.radius) >= self.min.x()
            && (sphere.center.y() - sphere.radius) >= self.min.y()
            && (sphere.center.z() - sphere.radius) >= self.min.z()
            && (sphere.center.x() + sphere.radius) <= self.max.x()
            && (sphere.center.y() + sphere.radius) <= self.max.y()
            && (sphere.center.z() + sphere.radius) <= self.max.z()
    }
}

impl Contains<Line> for Aabb3 {
    #[inline]
    fn contains(&self, line: &Line) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl Union for Aabb3 {
    type Output = Aabb3;

    fn union(&self, other: &Aabb3) -> Aabb3 {
        self.grow(other.min()).grow(other.max())
    }
}

impl Union<Sphere> for Aabb3 {
    type Output = Aabb3;

    fn union(&self, sphere: &Sphere) -> Aabb3 {
        self.grow(Vec3::new(
            sphere.center.x() - sphere.radius,
            sphere.center.y() - sphere.radius,
            sphere.center.z() - sphere.radius,
        )).grow(sphere.center + Vec3::splat(sphere.radius))
    }
}

impl Continuous<Aabb3> for Ray {
    type Result = Vec3;

    fn intersection(&self, aabb: &Aabb3) -> Option<Vec3> {
        let ray = self;
        
        let inv_dir = Vec3::new(1.0, 1.0, 1.0) / ray.direction;

        let mut t1 = (aabb.min.x() - ray.origin.x()) * inv_dir.x();
        let mut t2 = (aabb.max.x() - ray.origin.x()) * inv_dir.x();
        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        t1 = (aabb.min.y() - ray.origin.y()) * inv_dir.y();
        t2 = (aabb.max.y() - ray.origin.y()) * inv_dir.y();
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));

        t1 = (aabb.min.z() - ray.origin.z()) * inv_dir.z();
        t2 = (aabb.max.z() - ray.origin.z()) * inv_dir.z();
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));

        if (tmin < 0.0 && tmax < 0.0) || tmax < tmin {
            None
        } else {
            let t = if tmin >= 0.0 { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

impl Continuous<Ray> for Aabb3 {
    type Result = Vec3;

    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        ray.intersection(self)
    }
}

impl Discrete<Aabb3> for Ray {
    fn intersects(&self, aabb: &Aabb3) -> bool {
        let ray = self;
        
        let inv_dir = Vec3::new(1.0, 1.0, 1.0) / ray.direction;

        let mut t1 = (aabb.min.x() - ray.origin.x()) * inv_dir.x();
        let mut t2 = (aabb.max.x() - ray.origin.x()) * inv_dir.x();

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        t1 = (aabb.min.y() - ray.origin.y()) * inv_dir.y();
        t2 = (aabb.max.y() - ray.origin.y()) * inv_dir.y();
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));

        t1 = (aabb.min.z() - ray.origin.z()) * inv_dir.z();
        t2 = (aabb.max.z() - ray.origin.z()) * inv_dir.z();
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));

        tmax >= tmin && (tmin >= 0.0 || tmax >= 0.0)
    }
}

impl Discrete<Ray> for Aabb3 {
    fn intersects(&self, ray: &Ray) -> bool {
        ray.intersects(self)
    }
}

impl Discrete<Aabb3> for Aabb3 {
    fn intersects(&self, aabb: &Aabb3) -> bool {
        let (a0, a1) = (self.min(), self.max());
        let (b0, b1) = (aabb.min(), aabb.max());

        a1.x() > b0.x() && a0.x() < b1.x() && a1.y() > b0.y() && a0.y() < b1.y() && a1.z() > b0.z() && a0.z() < b1.z()
    }
}

impl PlaneBound for Aabb3 {
    fn relate_plane(&self, plane: Plane) -> Relation {
        let corners = self.to_corners();
        let first = corners[0].relate_plane(plane);
        for p in corners[1..].iter() {
            if p.relate_plane(plane) != first {
                return Relation::Cross;
            }
        }
        first
    }
}

impl SurfaceArea for Aabb3 {

    fn surface_area(&self) -> f32 {
        let dim = self.dim();
        2.0 * ((dim.x() * dim.y()) + (dim.x() * dim.z()) + (dim.y() * dim.z()))
    }
}