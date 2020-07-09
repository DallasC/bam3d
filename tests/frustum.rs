extern crate glam;
extern crate bam3d;

use glam::{Vec3, Mat4};
use bam3d::{Projection, Relation, Sphere};

#[test]
fn test_contains() {
    let frustum = Mat4::perspective_rh_gl(1_f32.to_radians(), 1., 1., 10.).perspective_to_frustum();
    assert_eq!(
        frustum.contains(&Sphere {
            center: Vec3::new(0f32, 0f32, -5f32),
            radius: 1f32,
        }),
        Relation::In
    );
    assert_eq!(
        frustum.contains(&Sphere {
            center: Vec3::new(0f32, 3f32, -5f32),
            radius: 1f32,
        }),
        Relation::Cross
    );
    assert_eq!(
        frustum.contains(&Sphere {
            center: Vec3::new(0f32, 0f32, 5f32),
            radius: 1f32,
        }),
        Relation::Out
    );
}