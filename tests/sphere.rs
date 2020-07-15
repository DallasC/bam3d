extern crate glam;
extern crate bam3d;

use glam::{Vec3};
use bam3d::*;

#[test]
fn test_intersection() {
    let sphere = Sphere {
        center: Vec3::new(0f32, 0f32, 0f32),
        radius: 1f32,
    };
    let r0 = Ray::new(
        Vec3::new(0f32, 0f32, 5f32),
        Vec3::new(0f32, 0f32, -5f32).normalize(),
    );
    let r1 = Ray::new(
        Vec3::new(1f32.cos(), 0f32, 5f32),
        Vec3::new(0f32, 0f32, -5f32).normalize(),
    );
    let r2 = Ray::new(
        Vec3::new(1f32, 0f32, 5f32),
        Vec3::new(0f32, 0f32, -5f32).normalize(),
    );
    let r3 = Ray::new(
        Vec3::new(2f32, 0f32, 5f32),
        Vec3::new(0f32, 0f32, -5f32).normalize(),
    );
    assert_eq!(
        sphere.intersection(&r0),
        Some(Vec3::new(0f32, 0f32, 1f32))
    );
    assert!(sphere.intersects(&r0));
    // TODO: z off a tiny bit. Floating point error?
    // assert_eq!(
    //     sphere.intersection(&r1).unwrap(),
    //     Vec3::new(1f32.cos(), 0f32, 1f32.sin())
    // );
    assert!(sphere.intersects(&r1));
    assert_eq!(
        sphere.intersection(&r2),
        Some(Vec3::new(1f32, 0f32, 0f32))
    );
    assert!(sphere.intersects(&r2));
    assert_eq!(sphere.intersection(&r3), None);
    assert!(!sphere.intersects(&r3));
}

#[test]
fn test_bound() {
    let point = Vec3::new(1f32, 2.0, 3.0);
    let sphere = Sphere {
        center: point,
        radius: 1.0,
    };
    let normal = Vec3::new(0f32, 0.0, 1.0);

    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point, normal)),
        Relation::Cross
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * -3.0, normal)),
        Relation::In
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * 3.0, normal)),
        Relation::Out
    );
}

#[test]
fn test_sphere_contains_point() {
    let sphere = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside_p = Vec3::new(1f32, 2.2, 3.);
    let outside_p = Vec3::new(11f32, 2.2, 3.);

    assert!(sphere.contains(&inside_p));
    assert!(!sphere.contains(&outside_p));
}

#[test]
fn test_sphere_contains_line() {
    let sphere = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Line::new(Vec3::new(1f32, 2., 3.), Vec3::new(1f32, 2.2, 3.));
    let outside = Line::new(Vec3::new(11f32, 2., 3.), Vec3::new(11f32, 2.2, 3.));
    let inside_out = Line::new(Vec3::new(1f32, 2., 3.), Vec3::new(11f32, 2.2, 3.));

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
}

#[test]
fn test_sphere_contains_sphere() {
    let sphere = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 0.1,
    };
    let outside = Sphere {
        center: Vec3::new(11f32, 2., 3.),
        radius: 0.1,
    };
    let inside_out = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 10.,
    };

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
}

#[test]
fn test_sphere_contains_aabb() {
    let sphere = Sphere {
        center: Vec3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Aabb3::new(Vec3::new(1f32, 2., 3.), Vec3::new(1f32, 2.2, 3.));
    let outside = Aabb3::new(Vec3::new(11f32, 2., 3.), Vec3::new(11f32, 2.2, 3.));
    let inside_out = Aabb3::new(Vec3::new(1f32, 2., 3.), Vec3::new(11f32, 2.2, 3.));
    let edge_case = Aabb3::new(Vec3::new(1f32, 1.01, 3.), Vec3::new(1.99f32, 2., 3.99));

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
    assert!(!sphere.contains(&edge_case));
}

#[test]
fn test_sphere_union_sphere() {
    let base = Sphere {
        center: Vec3::new(0., 0., 0.),
        radius: 5.,
    };

    let inside = Sphere {
        center: Vec3::new(1., 1., 1.),
        radius: 1.,
    };
    let outside = Sphere {
        center: Vec3::new(8., 8., 8.),
        radius: 1.,
    };
    let inside_out = Sphere {
        center: Vec3::new(4., 4., 4.),
        radius: 3.,
    };

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Sphere {
            center: Vec3::new(2.845_299_7, 2.845_299_7, 2.845_299_7),
            radius: 9.928_204,
        },
        base.union(&outside)
    );
    assert_eq!(
        Sphere {
            center: Vec3::new(1.422_649_9, 1.422_649_9, 1.422_649_9),
            radius: 7.464_102,
        },
        base.union(&inside_out)
    );
}

#[test]
fn test_sphere_union_aabb3() {
    let base = Sphere {
        center: Vec3::new(0., 0., 0.),
        radius: 5.,
    };

    let inside = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(2., 2., 2.));
    let outside = Aabb3::new(Vec3::new(6., 6., 6.), Vec3::new(9., 9., 9.));
    let inside_out = Aabb3::new(Vec3::new(4., 4., 4.), Vec3::new(7., 7., 7.));

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Sphere {
            center: Vec3::new(3.056_624_2, 3.056_624_2, 3.056_624_2),
            radius: 10.294_229,
        },
        base.union(&outside)
    );
    assert_eq!(
        Sphere {
            center: Vec3::new(2.056_624_2, 2.056_624_2, 2.056_624_2),
            radius: 8.562_178,
        },
        base.union(&inside_out)
    );
}

#[test]
fn test_surface_area() {
    let base = Sphere {
        center: Vec3::new(2., 1., -4.),
        radius: 2.,
    };

    // 4 * pi * r^2
    assert_eq!(4. * std::f32::consts::PI * 2. * 2., base.surface_area());
}