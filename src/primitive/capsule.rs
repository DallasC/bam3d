use glam::{Vec3, Mat4};

use crate::{Aabb3, Ray};
use crate::primitive::{Primitive, util::cylinder_ray_quadratic_solve};
use crate::volume::Sphere;

pub use crate::bound::{PlaneBound, Relation};
pub use crate::traits::*;
pub use crate::volume::{Aabb, MinMax};

/// Capsule primitive
/// Capsule body is aligned with the Y axis, with local origin in the center of the capsule.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Capsule {
    half_height: f32,
    radius: f32,
}

impl Capsule {
    /// Create a new Capsule
    pub fn new(half_height: f32, radius: f32) -> Self {
        Self {
            half_height,
            radius,
        }
    }

    /// Get radius
    pub fn radius(&self) -> f32 {
        self.radius
    }

    /// Get height
    pub fn height(&self) -> f32 {
        self.half_height + self.half_height
    }
}

impl Primitive for Capsule {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        //todo
        let direction = transform.inverse().transform_vector3(*direction);

        let mut result = Vec3::zero();
        result.set_y(direction.y().signum() * self.half_height);
        transform.transform_point3(result + direction * (self.radius / (direction.dot(direction)).sqrt()))
    }
}

impl ComputeBound<Aabb3> for Capsule {
    fn compute_bound(&self) -> Aabb3 {
        Aabb3::new(
            Vec3::new(-self.radius, -self.half_height - self.radius, -self.radius),
            Vec3::new(self.radius, self.half_height + self.radius, self.radius),
        )
    }
}

impl ComputeBound<Sphere> for Capsule {
    fn compute_bound(&self) -> Sphere {
        Sphere {
            center: Vec3::zero(),
            radius: self.half_height + self.radius,
        }
    }
}

impl Discrete<Ray> for Capsule {
    fn intersects(&self, r: &Ray) -> bool {
        let (t1, t2) = match cylinder_ray_quadratic_solve(r, self.radius) {
            None => return false,
            Some(t) => t,
        };

        if t1 < 0. && t2 < 0. {
            return false;
        }

        let t = if t1 < 0. {
            t2
        } else if t2 < 0. {
            t1
        } else {
            t1.min(t2)
        };

        let pc = r.origin + r.direction * t;
        if pc.y() <= self.half_height && pc.y() >= -self.half_height {
            return true;
        }

        // top sphere
        let l = Vec3::new(-r.origin.x(), self.half_height - r.origin.y(), -r.origin.z());
        let tca = l.dot(r.direction);
        if tca > 0. {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                return true;
            }
        }

        // bottom sphere
        let l = Vec3::new(-r.origin.x(), -self.half_height - r.origin.y(), -r.origin.z());
        let tca = l.dot(r.direction);
        if tca > 0. {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                return true;
            }
        }

        false
    }
}

impl Continuous<Ray> for Capsule {
    type Result = Vec3;

    fn intersection(&self, r: &Ray) -> Option<Vec3> {
        let (t1, t2) = match cylinder_ray_quadratic_solve(r, self.radius) {
            None => return None,
            Some(t) => t,
        };

        if t1 < 0. && t2 < 0. {
            return None;
        }

        let mut t = if t1 < 0. {
            t2
        } else if t2 < 0. {
            t1
        } else {
            t1.min(t2)
        };

        // top sphere
        let l = Vec3::new(-r.origin.x(), self.half_height - r.origin.y(), -r.origin.z());
        let tca = l.dot(r.direction);
        if tca > 0. {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                let thc = (self.radius * self.radius - d2).sqrt();
                let t0 = tca - thc;
                if t0 >= 0. && (t.is_nan() || t0 < t) {
                    t = t0;
                }
            }
        }

        // bottom sphere
        let l = Vec3::new(-r.origin.x(), -self.half_height - r.origin.y(), -r.origin.z());
        let tca = l.dot(r.direction);
        if tca > 0. {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                let thc = (self.radius * self.radius - d2).sqrt();
                let t0 = tca - thc;
                if t0 >= 0. && (t.is_nan() || t0 < t) {
                    t = t0;
                }
            }
        }

        if t.is_nan() {
            return None;
        }

        let pc = r.origin + r.direction * t;
        let full_half_height = self.half_height + self.radius;
        if (pc.y() > full_half_height) || (pc.y() < -full_half_height) {
            None
        } else {
            Some(pc)
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{Vec3, Mat4, Quat};

    use super::*;

    #[test]
    fn test_capsule_aabb() {
        let capsule = Capsule::new(2., 1.);
        assert_eq!(
            Aabb3::new(Vec3::new(-1., -3., -1.), Vec3::new(1., 3., 1.)),
            capsule.compute_bound()
        );
    }

    #[test]
    fn test_capsule_support_1() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert!(Vec3::new(1., 2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_capsule_support_2() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vec3::new(0.5, -1., 0.).normalize();
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert!(Vec3::new(0.447_213_65, -2.894_427_3, 0.).cmpeq(point).all());
    }

    #[test]
    fn test_capsule_support_3() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vec3::new(0., 1., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert!(Vec3::new(0., 3., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_capsule_support_4() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(10., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert!(Vec3::new(11., 2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_capsule_support_5() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., std::f32::consts::PI);
        let point = capsule.support_point(&direction, &transform);
        assert!(Vec3::new(1., -2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_discrete_1() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_2() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(-1., 0., 0.));
        assert!(!capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_3() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, -1., 0.1).normalize(),
        );
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_4() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, 1., 0.1).normalize(),
        );
        assert!(!capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_5() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0., -1., 0.).normalize(),
        );
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_continuous_1() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(Some(Vec3::new(-1., 0., 0.)), capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_2() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(None, capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_3() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 4., 0.),
            Vec3::new(0.1, -1., 0.1).normalize(),
        );
        assert_eq!(
            Some(Vec3::new(
                0.101_025_885_148_699_44,
                2.989_741_148_513_006,
                0.101_025_885_148_699_44
            )),
            capsule.intersection(&ray)
        );
    }

    #[test]
    fn test_continuous_4() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, 1., 0.1).normalize(),
        );
        assert_eq!(None, capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_5() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 4., 0.),
            Vec3::new(0., -1., 0.).normalize(),
        );
        assert_eq!(Some(Vec3::new(0., 3., 0.)), capsule.intersection(&ray));
    }

    // util
    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_z(rot.to_radians());
        let tran = Vec3::new(dx, dy, dz);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }
}