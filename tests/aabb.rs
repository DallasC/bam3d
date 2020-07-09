extern crate glam;
extern crate bam3d;

use glam::{Vec3, Mat4, Quat};
use bam3d::{Aabb, Aabb3};
use bam3d::{Contains, Continuous, Discrete, SurfaceArea, Union};
use bam3d::{Plane, PlaneBound, Ray, Sphere, Relation, Line};

#[test]
fn test_general() {
    let aabb = Aabb3::new(
        Vec3::new(-20., 30., 5.),
        Vec3::new(10., -10., -5.),
    );
    assert_eq!(aabb.min(), Vec3::new(-20., -10., -5.));
    assert_eq!(aabb.max(), Vec3::new(10., 30., 5.));
    assert_eq!(aabb.dim(), Vec3::new(30., 40., 10.));
    assert_eq!(aabb.volume(), 30. * 40. * 10.);
    assert_eq!(aabb.center(), Vec3::new(-5., 10., 0.));

    assert!(aabb.contains(&Vec3::new(0., 0., 0.)));
    assert!(!aabb.contains(&Vec3::new(-100., 0., 0.)));
    assert!(!aabb.contains(&Vec3::new(100., 0., 0.)));
    assert!(aabb.contains(&Vec3::new(9., 29., -1.)));
    assert!(!aabb.contains(&Vec3::new(10., 30., 5.)));
    assert!(aabb.contains(&Vec3::new(-20., -10., -5.)));
    assert!(!aabb.contains(&Vec3::new(-21., -11., -6.)));

    assert_eq!(
        aabb.add_v(Vec3::new(1., 2., 3.)),
        Aabb3::new(
            Vec3::new(-19., 32., 8.),
            Vec3::new(11., -8., -2.),
        )
    );

    assert_eq!(
        aabb.mul_s(2.),
        Aabb3::new(
            Vec3::new(-40., -20., -10.),
            Vec3::new(20., 60., 10.),
        )
    );

    assert_eq!(
        aabb.mul_v(Vec3::new(1., 2., 3.)),
        Aabb3::new(
            Vec3::new(-20., -20., -15.),
            Vec3::new(10., 60., 15.),
        )
    );

    assert_eq!(
        aabb.add_margin(Vec3::new(2., 2., 2.)),
        Aabb3::new(
            Vec3::new(-22., -12., -7.),
            Vec3::new(12., 32., 7.),
        )
    );
}

#[test]
fn test_parallel_ray_should_not_intersect() {
    let aabb = Aabb3::new(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(Vec3::new(0.0f32, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let ray_y = Ray::new(Vec3::new(0.0f32, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    let ray_z = Ray::new(Vec3::new(0.0f32, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let ray_z_imprecise = Ray::new(
        Vec3::new(0.0f32, 0.0, 0.0),
        Vec3::new(0.0001, 0.0001, 1.0),
    );

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));
    assert_eq!(ray_z_imprecise.intersection(&aabb), None);
    assert!(!ray_z_imprecise.intersects(&aabb));
}

#[test]
fn test_oblique_ray_should_intersect() {
    let aabb = Aabb3::new(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 5.0, 5.0));
    let ray1 = Ray::new(
        Vec3::new(0.0f32, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 1.0).normalize(),
    );
    let ray2 = Ray::new(Vec3::new(0.0f32, 6.0, 0.0), Vec3::new(1.0, -1.0, 1.0));

    assert_eq!(ray1.intersection(&aabb), Some(Vec3::new(1.0, 1.0, 1.0)));
    assert!(ray1.intersects(&aabb));
    assert_eq!(ray2.intersection(&aabb), Some(Vec3::new(1.0, 5.0, 1.0)));
    assert!(ray2.intersects(&aabb));
}

#[test]
fn test_pointing_to_other_dir_ray_should_not_intersect() {
    let aabb = Aabb3::new(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(
        Vec3::new(0.0f32, 2.0, 2.0),
        Vec3::new(-1.0, 0.01, 0.01),
    );
    let ray_y = Ray::new(
        Vec3::new(2.0f32, 0.0, 2.0),
        Vec3::new(0.01, -1.0, 0.01),
    );
    let ray_z = Ray::new(
        Vec3::new(2.0f32, 2.0, 0.0),
        Vec3::new(0.01, 0.01, -1.0),
    );

    let ray_x2 = Ray::new(Vec3::new(6.0f32, 2.0, 2.0), Vec3::new(1.0, 0.0, 0.0));
    let ray_y2 = Ray::new(Vec3::new(2.0f32, 6.0, 2.0), Vec3::new(0.0, 1.0, 0.0));
    let ray_z2 = Ray::new(Vec3::new(2.0f32, 2.0, 6.0), Vec3::new(0.0, 0.0, 1.0));

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), None);
    assert!(!ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), None);
    assert!(!ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), None);
    assert!(!ray_z2.intersects(&aabb));
}

#[test]
fn test_pointing_to_box_dir_ray_should_intersect() {
    let aabb = Aabb3::new(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(Vec3::new(0.0f32, 2.0, 2.0), Vec3::new(1.0, 0.0, 0.0));
    let ray_y = Ray::new(Vec3::new(2.0f32, 0.0, 2.0), Vec3::new(0.0, 1.0, 0.0));
    let ray_z = Ray::new(Vec3::new(2.0f32, 2.0, 0.0), Vec3::new(0.0, 0.0, 1.0));

    let ray_x2 = Ray::new(Vec3::new(6.0f32, 2.0, 2.0), Vec3::new(-1.0, 0.0, 0.0));
    let ray_y2 = Ray::new(Vec3::new(2.0f32, 6.0, 2.0), Vec3::new(0.0, -1.0, 0.0));
    let ray_z2 = Ray::new(Vec3::new(2.0f32, 2.0, 6.0), Vec3::new(0.0, 0.0, -1.0));

    assert_eq!(ray_x.intersection(&aabb), Some(Vec3::new(1.0, 2.0, 2.0)));
    assert!(ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), Some(Vec3::new(2.0, 1.0, 2.0)));
    assert!(ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), Some(Vec3::new(2.0, 2.0, 1.0)));
    assert!(ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), Some(Vec3::new(5.0, 2.0, 2.0)));
    assert!(ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), Some(Vec3::new(2.0, 5.0, 2.0)));
    assert!(ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), Some(Vec3::new(2.0, 2.0, 5.0)));
    assert!(ray_z2.intersects(&aabb));
}

#[test]
fn test_corners() {
    let corners = Aabb3::new(
        Vec3::new(-20., 30., 5.),
        Vec3::new(10., -10., -5.),
    ).to_corners();
    assert!(corners.contains(&Vec3::new(-20., 30., -5.)));
    assert!(corners.contains(&Vec3::new(10., 30., 5.)));
    assert!(corners.contains(&Vec3::new(10., -10., 5.)));
}

#[test]
fn test_bound() {
    let aabb = Aabb3::new(Vec3::new(-5.0f32, 5.0, 0.0), Vec3::new(5.0, 10.0, 1.0));
    let plane1 =
        Plane::from_point_normal(Vec3::new(0f32, 0.0, 0.0), Vec3::new(0f32, 0.0, 1.0));
    let plane2 =
        Plane::from_point_normal(Vec3::new(-5.0f32, 4.0, 0.0), Vec3::new(0f32, 1.0, 0.0));
    let plane3 =
        Plane::from_point_normal(Vec3::new(6.0f32, 0.0, 0.0), Vec3::new(1f32, 0.0, 0.0));
    assert_eq!(aabb.relate_plane(plane1), Relation::Cross);
    assert_eq!(aabb.relate_plane(plane2), Relation::In);
    assert_eq!(aabb.relate_plane(plane3), Relation::Out);
}

#[test]
fn test_aab3_should_not_intersect() {
    use bam3d::Discrete;
    let a = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));
    let b = Aabb3::new(Vec3::new(15., 15., 15.), Vec3::new(25., 25., 25.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aab3_should_intersect() {
    use bam3d::Discrete;
    let a = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));
    let b = Aabb3::new(Vec3::new(5., 5., 5.), Vec3::new(15., 15., 15.));
    assert!(a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_x() {
    use bam3d::Discrete;
    let a = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));
    let b = Aabb3::new(Vec3::new(5., 11., 11.), Vec3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_y() {
    use bam3d::Discrete;
    let a = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));
    let b = Aabb3::new(Vec3::new(11., 5., 11.), Vec3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_z() {
    use bam3d::Discrete;
    let a = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));
    let b = Aabb3::new(Vec3::new(11., 11., 5.), Vec3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_contains_vec3() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Vec3::new(3., 3., 3.);
    let outside = Vec3::new(11., 11., 11.);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));

    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Vec3::new(3., 3., 3.);
    let outside = Vec3::new(11., 11., 11.);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
}

#[test]
fn test_aabb3_contains_line3() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Line::new(Vec3::new(1., 1., 1.), Vec3::new(2., 2., 2.));
    let inside_out = Line::new(Vec3::new(1., 1., 1.), Vec3::new(12., 12., 12.));
    let outside = Line::new(Vec3::new(11., 11., 11.), Vec3::new(12., 12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_aabb3() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Aabb3::new(Vec3::new(1., 1., 1.), Vec3::new(2., 2., 2.));
    let inside_out = Aabb3::new(Vec3::new(1., 1., 1.), Vec3::new(12., 12., 12.));
    let outside = Aabb3::new(Vec3::new(11., 11., 11.), Vec3::new(12., 12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_sphere() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Sphere {
        center: Vec3::new(5., 5., 5.),
        radius: 1.,
    };

    let inside_out = Sphere {
        center: Vec3::new(5., 5., 5.),
        radius: 10.,
    };
    let outside = Sphere {
        center: Vec3::new(20., 20., 20.),
        radius: 1.,
    };

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_ray_aabb3_parallel() {
    let aabb = Aabb3::new(Vec3::new(5., 5., 5.), Vec3::new(10., 10., 10.));

    let ray = Ray::new(Vec3::new(2., 0., 2.), Vec3::new(0., 1., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray::new(Vec3::new(0., 2., 2.), Vec3::new(1., 0., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray::new(Vec3::new(2., 2., 0.), Vec3::new(0., 0., 1.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());
}

#[test]
fn test_aabb3_union_aabb3() {
    let base = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Aabb3::new(Vec3::new(2., 2., 2.), Vec3::new(5., 5., 5.));
    let outside = Aabb3::new(Vec3::new(12., 12., 12.), Vec3::new(15., 15., 15.));
    let inside_out = Aabb3::new(Vec3::new(2., 2., 2.), Vec3::new(12., 12., 12.));

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(15., 15., 15.)),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(12., 12., 12.)),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_union_sphere() {
    let base = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 10., 10.));

    let inside = Sphere {
        center: Vec3::new(5., 5., 5.),
        radius: 1.,
    };
    let outside = Sphere {
        center: Vec3::new(14., 14., 14.),
        radius: 1.,
    };
    let inside_out = Sphere {
        center: Vec3::new(8., 8., 8.),
        radius: 4.,
    };

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(15., 15., 15.)),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(12., 12., 12.)),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_surface_area() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 20., 30.));
    // 2 * (x*y + x*z + y*z)
    assert_eq!(2200., aabb.surface_area());
}

#[test]
fn test_aabb3_transform() {
    let aabb = Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(10., 20., 30.));
    let transform = Mat4::from_scale_rotation_translation(Vec3::splat(1.), Quat::from_rotation_z(0.0_f32.to_radians()), Vec3::new(5., 3., 1.));
    assert_eq!(
        Aabb3::new(Vec3::new(5., 3., 1.), Vec3::new(15., 23., 31.)),
        aabb.transform(&transform)
    );
}