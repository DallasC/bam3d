//! Bounding sphere

use glam::{Vec3, Mat4};

use crate::{Aabb3, line::Line, plane::Plane, ray::Ray, bound::*, traits::*, volume::Aabb};

/// Bounding sphere.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Sphere {
    /// Center point of the sphere in world space
    pub center: Vec3,
    /// Sphere radius
    pub radius: f32,
}

impl Bound for Sphere {

    fn min_extent(&self) -> Vec3 {
        self.center + Vec3::splat(-self.radius)
    }

    fn max_extent(&self) -> Vec3 {
        self.center + Vec3::splat(self.radius)
    }

    fn with_margin(&self, add: Vec3) -> Self {
        let max = add.x().max(add.y()).max(add.z());
        Sphere {
            center: self.center,
            radius: self.radius + max,
        }
    }

    fn transform_volume(&self, transform: &Mat4) -> Self {
        Sphere {
            center: transform.transform_point3(self.center),
            radius: self.radius,
        }
    }

    fn empty() -> Self {
        Self {
            center: Vec3::zero(),
            radius: 0.,
        }
    }
}

impl Continuous<Ray> for Sphere {
    type Result = Vec3;
    fn intersection(&self, r: &Ray) -> Option<Vec3> {
        let s = self;

        let l = s.center - r.origin;
        let tca = l.dot(r.direction);
        if tca < 0. {
            return None;
        }
        let d2 = l.dot(l) - tca * tca;
        if d2 > s.radius * s.radius {
            return None;
        }
        let thc = (s.radius * s.radius - d2).sqrt();
        Some(r.origin + r.direction * (tca - thc))
    }
}

impl Discrete<Ray> for Sphere {
    fn intersects(&self, r: &Ray) -> bool {
        let s = self;
        let l = s.center - r.origin;
        let tca = l.dot(r.direction);
        if tca < 0. {
            return false;
        }
        let d2 = l.dot(l) - tca * tca;
        d2 <= s.radius * s.radius
    }
}

impl Discrete<Sphere> for Sphere {
    fn intersects(&self, s2: &Sphere) -> bool {
        let s1 = self;

        let distance = s1.center - s2.center;
        let radiuses = s1.radius + s2.radius;

        (distance).dot(distance) <= radiuses * radiuses
    }
}

impl PlaneBound for Sphere {
    fn relate_plane(&self, plane: Plane) -> Relation {
        let dist = self.center.dot(plane.n) - plane.d;
        if dist > self.radius {
            Relation::In
        } else if dist < -self.radius {
            Relation::Out
        } else {
            Relation::Cross
        }
    }
}

impl Contains<Aabb3> for Sphere {
    // will return true for border hits
    #[inline]
    fn contains(&self, aabb: &Aabb3) -> bool {
        let radius_sq = self.radius * self.radius;
        for c in &aabb.to_corners() {
            let distance = *c - self.center;
            if (distance).dot(distance) > radius_sq {
                return false;
            }
        }
        true
    }
}

impl Contains<Vec3> for Sphere {
    #[inline]
    fn contains(&self, p: &Vec3) -> bool {
        let distance = *p - self.center;
        (distance).dot(distance) <= self.radius * self.radius
    }
}

impl Contains<Line> for Sphere {
    #[inline]
    fn contains(&self, line: &Line) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl Contains<Sphere> for Sphere {
    #[inline]
    fn contains(&self, other: &Sphere) -> bool {
        let center_dist = other.center - self.center;
        let distance_sq = (center_dist).dot(center_dist);
        distance_sq.sqrt() + other.radius <= self.radius
    }
}

impl Union for Sphere {
    type Output = Sphere;

    fn union(&self, other: &Sphere) -> Sphere {
        if self.contains(other) {
            return *self;
        }
        if other.contains(self) {
            return *other;
        }
        let center_diff = other.center - self.center;
        let center_diff_s = (center_diff.dot(center_diff)).sqrt();
        let radius = (self.radius + other.radius + center_diff_s) / 2.;
        Sphere {
            radius,
            center: self.center + center_diff * (radius - self.radius) / center_diff_s,
        }
    }
}

impl Union<Aabb3> for Sphere {
    type Output = Sphere;

    fn union(&self, aabb: &Aabb3) -> Sphere {
        if self.contains(aabb) {
            return *self;
        }
        let dist = aabb.max() - aabb.center();
        let dist_sq = dist.dot(dist);
        let aabb_radius = dist_sq.sqrt();
        if aabb.contains(self) {
            return Sphere {
                center: aabb.center(),
                radius: aabb_radius,
            };
        }
        let center_diff = aabb.center() - self.center;
        let center_sq = center_diff.dot(center_diff);
        let center_diff_s = center_sq.sqrt();
        let radius = (self.radius + aabb_radius + center_diff_s) / 2.;
        Sphere {
            center: self.center + center_diff * (radius - self.radius) / center_diff_s,
            radius,
        }
    }
}

impl SurfaceArea for Sphere {

    fn surface_area(&self) -> f32 {
        4. * std::f32::consts::PI * self.radius * self.radius
    }
}