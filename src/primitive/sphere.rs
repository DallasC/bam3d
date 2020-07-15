use glam::{Vec3, Mat4};

use crate::{Aabb3, ray::Ray, traits::*};

/// Sphere primitive
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Sphere {
    /// Radius of the sphere
    pub radius: f32,
}

impl Sphere {
    /// Create a new sphere primitive
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
}

impl Primitive for Sphere {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        let direction = transform.inverse().transform_vector3(*direction);
        transform.transform_point3(direction * (self.radius/(direction.dot(direction)).sqrt()))
    }
}

impl ComputeBound<Aabb3> for Sphere {
    fn compute_bound(&self) -> Aabb3 {
        Aabb3::new(
            Vec3::splat(-self.radius),
            Vec3::splat(self.radius),
        )
    }
}

impl ComputeBound<crate::volume::Sphere> for Sphere {
    fn compute_bound(&self) -> crate::volume::Sphere {
        crate::volume::Sphere {
            center: Vec3::zero(),
            radius: self.radius,
        }
    }
}

impl Discrete<Ray> for Sphere {
    fn intersects(&self, r: &Ray) -> bool {
        let s = self;
        let l = Vec3::new(-r.origin.x(), -r.origin.y(), -r.origin.z());
        let tca = l.dot(r.direction);
        if tca < 0. {
            return false;
        }
        let d2 = l.dot(l) - tca * tca;
        d2 <= s.radius * s.radius
    }
}

impl Continuous<Ray> for Sphere {
    type Result = Vec3;

    fn intersection(&self, r: &Ray) -> Option<Vec3> {
        let s = self;

        let l = Vec3::new(-r.origin.x(), -r.origin.y(), -r.origin.z());
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

#[cfg(test)]
mod tests {
    use glam::{Vec3, Mat4, Quat};

    use super::*;

    // sphere
    #[test]
    fn test_sphere_support_1() {
        test_sphere_support(1., 0., 0., 10., 0., 0., 0.);
    }

    #[test]
    fn test_sphere_support_2() {
        test_sphere_support(
            1.,
            1.,
            1.,
            5.773_503,
            5.773_503,
            5.773_503,
            0.,
        );
    }

    #[test]
    fn test_sphere_support_3() {
        test_sphere_support(
            1.,
            0.,
            0.,
            10.,
            0.,
            0.,
            -std::f32::consts::PI / 4.,
        );
    }

    #[test]
    fn test_sphere_support_4() {
        let sphere = Sphere::new(10.);
        let direction = Vec3::new(1., 0., 0.);
        let t = transform(0., 10., 0., 0.);
        let point = sphere.support_point(&direction, &t);
        assert_eq!(Vec3::new(10., 10., 0.), point);
    }

    #[test]
    fn test_sphere_bound() {
        let sphere = Sphere::new(10.);
        assert_eq!(
            bound(-10., -10., -10., 10., 10., 10.),
            sphere.compute_bound()
        )
    }

    #[test]
    fn test_ray_discrete() {
        let sphere = Sphere::new(10.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        assert!(sphere.intersects(&ray));
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(!sphere.intersects(&ray));
        let ray = Ray::new(Vec3::new(20., -15., 0.), Vec3::new(-1., 0., 0.));
        assert!(!sphere.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.);
        assert!(sphere.intersects_transformed(&ray, &t));
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &t));
        let t = transform(0., 15., 0., 0.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let sphere = Sphere::new(10.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(Some(Vec3::new(10., 0., 0.)), sphere.intersection(&ray));
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection(&ray));
        let ray = Ray::new(Vec3::new(20., -15., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.);
        assert_eq!(
            Some(Vec3::new(10., 0., 0.)),
            sphere.intersection_transformed(&ray, &t)
        );
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &t));
        let t = transform(0., 15., 0., 0.);
        let ray = Ray::new(Vec3::new(20., 0., 0.), Vec3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &t));
    }

    fn test_sphere_support(dx: f32, dy: f32, dz: f32, px: f32, py: f32, pz: f32, rot: f32) {
        let sphere = Sphere::new(10.);
        let direction = Vec3::new(dx, dy, dz);
        let t = transform(0., 0., 0., rot);
        let point = sphere.support_point(&direction, &t);
        assert_eq!(px, point.x());
        assert_eq!(py, point.y());
        assert_eq!(pz, point.z());
    }

    // util

    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_z(rot.to_radians());
        let tran = Vec3::new(dx, dy, dz);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }

    fn bound(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) -> Aabb3 {
        Aabb3::new(
            Vec3::new(min_x, min_y, min_z),
            Vec3::new(max_x, max_y, max_z),
        )
    }
}