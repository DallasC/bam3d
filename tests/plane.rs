extern crate glam;
extern crate bam3d;

use glam::{Vec3};
use bam3d::*;

#[test]
fn test_from_points() {
    assert_eq!(
        Plane::from_points(
            Vec3::new(5.0f32, 0.0f32, 5.0f32),
            Vec3::new(5.0f32, 5.0f32, 5.0f32),
            Vec3::new(5.0f32, 0.0f32, -1.0f32),
        ),
        Some(Plane::from_abcd(-1.0f32, 0.0f32, 0.0f32, 5.0f32))
    );

    assert_eq!(
        Plane::from_points(
            Vec3::new(0.0f32, 5.0f32, -5.0f32),
            Vec3::new(0.0f32, 5.0f32, 0.0f32),
            Vec3::new(0.0f32, 5.0f32, 5.0f32),
        ),
        None
    ); // The points are parallel
}

#[test]
fn test_ray_intersection() {
    let p0 = Plane::from_abcd(1f32, 0f32, 0f32, -7f32);
    let r0: Ray = Ray::new(
        Vec3::new(2f32, 3f32, 4f32),
        Vec3::new(1f32, 1f32, 1f32).normalize(),
    );
    assert!(p0.intersects(&r0));
    assert_eq!(p0.intersection(&r0), Some(Vec3::new(7f32, 8f32, 9f32)));

    let p1 = Plane::from_points(
        Vec3::new(5f32, 0f32, 5f32),
        Vec3::new(5f32, 5f32, 5f32),
        Vec3::new(5f32, 0f32, -1f32),
    ).unwrap();
    let r1: Ray = Ray::new(
        Vec3::new(0f32, 0f32, 0f32),
        Vec3::new(-1f32, 0f32, 0f32).normalize(),
    );
    assert_eq!(p1.intersection(&r1), None); // r1 points away from p1
    assert!(!p1.intersects(&r1));
}

#[test]
fn test_plane2_intersection() {
    let p0 = Plane::new(Vec3::unit_x(), 1.0f32);
    let p1 = Plane::new(Vec3::unit_y(), 2.0f32);
    let ray = p0.intersection(&p1);
    assert!(ray.is_some());
    assert!(p0.intersects(&p1));

    let ray = ray.unwrap();
    assert_eq!(ray.origin.x(), 1.0f32);
    assert_eq!(ray.origin.y(), 2.0f32);
    assert_eq!(ray.origin.z(), 0.0f32);
    assert_eq!(ray.direction.x(), 0.0f32);
    assert_eq!(ray.direction.y(), 0.0f32);
    assert_eq!(ray.direction.z(), 1.0f32);

    let p0 = Plane::new(Vec3::unit_y(), 1.0f32);
    let p1 = Plane::new(Vec3::unit_y(), 2.0f32);
    let ray = p0.intersection(&p1);
    assert!(ray.is_none());
    assert!(!p0.intersects(&p1));
}

#[test]
fn test_plane3_intersection() {
    let p0 = Plane::new(Vec3::unit_x(), 1.0f32);
    let p1 = Plane::new(Vec3::unit_y(), 2.0f32);
    let p2 = Plane::new(Vec3::unit_z(), 3.0f32);
    let point = p0.intersection(&(p1, p2));
    assert!(point.is_some());
    assert!(p0.intersects(&(p1, p2)));

    let point = point.unwrap();
    assert_eq!(point.x(), 1.0f32);
    assert_eq!(point.y(), 2.0f32);
    assert_eq!(point.z(), 3.0f32);

    let p0 = Plane::new(Vec3::unit_y(), 1.0f32);
    let p1 = Plane::new(Vec3::unit_y(), 2.0f32);
    let p2 = Plane::new(Vec3::unit_z(), 3.0f32);
    let point = p0.intersection(&(p1, p2));
    assert!(point.is_none());
    assert!(!p0.intersects(&(p1, p2)));
}