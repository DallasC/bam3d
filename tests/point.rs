extern crate glam;
extern crate bam3d;

use glam::{Vec3};
use bam3d::{Continuous, Plane, PlaneBound, Ray, Relation};

#[test]
fn test_bound() {
    let point = Vec3::new(1f32, 2.0, 3.0);
    let normal = Vec3::new(0f32, -0.8, -0.36);
    let plane = Plane::from_point_normal(point, normal);

    assert_eq!(point.relate_plane(plane), Relation::Cross);
    assert_eq!((point + normal).relate_plane(plane), Relation::In);
    assert_eq!((point + normal * -1.0).relate_plane(plane), Relation::Out);
}

#[test]
fn test_ray_intersect() {
    let point = Vec3::new(1f32, 2.0, 3.0);
    // ray across the point
    let ray1 = Ray::new(Vec3::new(1f32, 2.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(point.intersection(&ray1), Some(point));
    // ray in the opposite direction
    let ray2 = Ray::new(Vec3::new(1f32, 2.0, 0.0), Vec3::new(0.0, 0.0, -1.0));
    assert_eq!(point.intersection(&ray2), None);
    // unrelated ray
    let ray3 = Ray::new(Vec3::new(1f32, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(point.intersection(&ray3), None);
}