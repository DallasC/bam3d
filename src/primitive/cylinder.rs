use glam::{Vec3, Mat4};

use crate::{ray::Ray, Aabb3, traits::*};
use crate::volume::Sphere;
use crate::primitive::{Primitive, util::cylinder_ray_quadratic_solve};


/// Cylinder primitive
/// Cylinder body is aligned with the Y axis, with local origin in the center of the cylinders.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cylinder {
    half_height: f32,
    radius: f32,
}

impl Cylinder {
    /// Create a new cylinder
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

impl Primitive for Cylinder {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        let direction = transform.inverse().transform_vector3(*direction);

        let mut result = direction;
        let negative = result.y().is_sign_negative();

        result.set_y(0.);
        if (result.dot(result) - 0.).abs() < std::f32::EPSILON {
            result = Vec3::splat(0.);
        } else {
            result = result.normalize();
            if (result.y() - 0.).abs() < std::f32::EPSILON && (result.x() - 0.).abs() < std::f32::EPSILON && (result.z() - 0.).abs() < std::f32::EPSILON {
                result = Vec3::splat(0.); // cancel out any inconsistencies
            } else {
                result *= self.radius;
            }
        }
        if negative {
            result.set_y(-self.half_height);
        } else {
            result.set_y(self.half_height);
        }
        transform.transform_point3(result)
    }
}

impl ComputeBound<Aabb3> for Cylinder {
    fn compute_bound(&self) -> Aabb3 {
        Aabb3::new(
            Vec3::new(-self.radius, -self.half_height, -self.radius),
            Vec3::new(self.radius, self.half_height, self.radius),
        )
    }
}

impl ComputeBound<Sphere> for Cylinder {
    fn compute_bound(&self) -> Sphere {
        Sphere {
            center: Vec3::zero(),
            radius: ((self.radius * self.radius) + (self.half_height * self.half_height)).sqrt(),
        }
    }
}

impl Discrete<Ray> for Cylinder {
    fn intersects(&self, r: &Ray) -> bool {
        if (r.direction.x() - 0.).abs() < std::f32::EPSILON && (r.direction.z() - 0.).abs() < std::f32::EPSILON {
            if (r.direction.y() - 0.).abs() < std::f32::EPSILON {
                return false;
            }

            return (r.origin.y() >= -self.half_height && r.direction.y() <= 0.)
                || (r.origin.y() <= self.half_height && r.direction.y() >= 0.);
        }

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

        let n = -Vec3::unit_y();
        let tp = -(self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tp >= 0. {
            let p = r.origin + r.direction * tp;
            if p.x() * p.x() + p.z() * p.z() < self.radius * self.radius {
                return true;
            }
        }

        let n = Vec3::unit_y();
        let tb = -(-self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tb >= 0. {
            let p = r.origin + r.direction * tb;
            if p.x() * p.x() + p.z() * p.z() < self.radius * self.radius {
                return true;
            }
        }

        false
    }
}

impl Continuous<Ray> for Cylinder {
    type Result = Vec3;

    fn intersection(&self, r: &Ray) -> Option<Vec3> {
        if (r.direction.x() - 0.).abs() < std::f32::EPSILON && (r.direction.z() - 0.).abs() < std::f32::EPSILON {
            if (r.direction.y() - 0.).abs() < std::f32::EPSILON {
                return None;
            }

            if r.origin.y() >= self.half_height && r.direction.y() < 0. {
                return Some(Vec3::new(0., self.half_height, 0.));
            }
            if r.origin.y() >= -self.half_height && r.direction.y() < 0. {
                return Some(Vec3::new(0., -self.half_height, 0.));
            }
            if r.origin.y() <= -self.half_height && r.direction.y() > 0. {
                return Some(Vec3::new(0., -self.half_height, 0.));
            }
            if r.origin.y() <= self.half_height && r.direction.y() > 0. {
                return Some(Vec3::new(0., self.half_height, 0.));
            }

            return None;
        }

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

        let n = -Vec3::unit_y();
        let tp = -(self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tp >= 0. && tp < t {
            let p = r.origin + r.direction * tp;
            if p.x() * p.x() + p.z() * p.z() < self.radius * self.radius {
                t = tp;
            }
        }

        let n = Vec3::unit_y();
        let tb = -(-self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tb >= 0. && tb < t {
            let p = r.origin + r.direction * tb;
            if p.x() * p.x() + p.z() * p.z() < self.radius * self.radius {
                t = tb;
            }
        }

        let pc = r.origin + r.direction * t;
        if (pc.y() > self.half_height) || (pc.y() < -self.half_height) {
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
    fn test_cylinder_aabb() {
        let cylinder = Cylinder::new(2., 1.);
        assert_eq!(
            Aabb3::new(Vec3::new(-1., -2., -1.), Vec3::new(1., 2., 1.)),
            cylinder.compute_bound()
        );
    }

    #[test]
    fn test_cylinder_support_1() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert!(Vec3::new(1., 2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_cylinder_support_2() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vec3::new(0.5, -1., 0.).normalize();
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert!(Vec3::new(1., -2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_cylinder_support_3() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vec3::new(0., 1., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert!(Vec3::new(0., 2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_cylinder_support_4() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(10., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert!(Vec3::new(11., 2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_cylinder_support_5() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vec3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., std::f32::consts::PI);
        let point = cylinder.support_point(&direction, &transform);
        assert!(Vec3::new(1., -2., 0.).cmpeq(point).all());
    }

    #[test]
    fn test_discrete_1() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_2() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(-1., 0., 0.));
        assert!(!cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_3() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, -1., 0.1).normalize(),
        );
        assert!(cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_4() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, 1., 0.1).normalize(),
        );
        assert!(!cylinder.intersects(&ray));
    }

    #[test]
    fn test_continuous_1() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(Some(Vec3::new(-1., 0., 0.)), cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_2() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(Vec3::new(-3., 0., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(None, cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_3() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, -1., 0.1).normalize(),
        );
        assert_eq!(Some(Vec3::new(0.1, 2., 0.1)), cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_4() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0.1, 1., 0.1).normalize(),
        );
        assert_eq!(None, cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_5() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray::new(
            Vec3::new(0., 3., 0.),
            Vec3::new(0., -1., 0.).normalize(),
        );
        assert_eq!(Some(Vec3::new(0., 2., 0.)), cylinder.intersection(&ray));
    }

    // util
    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_z(rot.to_radians());
        let tran = Vec3::new(dx, dy, dz);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }
}