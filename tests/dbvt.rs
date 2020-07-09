extern crate glam;
extern crate bam3d;
extern crate rand;

use glam::{Vec3, Mat4};
use bam3d::{Aabb3, Frustum, Projection, Relation};
use bam3d::dbvt::*;
use bam3d::prelude::*;

#[derive(Debug, Clone)]
struct Value3 {
    pub id: u32,
    pub aabb: Aabb3,
    fat_aabb: Aabb3,
}

impl Value3 {
    pub fn new(id: u32, aabb: Aabb3) -> Self {
        Self {
            id,
            fat_aabb: aabb.add_margin(Vec3::new(3., 3., 3.)),
            aabb,
        }
    }
}

impl TreeValue for Value3 {
    type Bound = Aabb3;

    fn bound(&self) -> &Aabb3 {
        &self.aabb
    }

    fn get_bound_with_margin(&self) -> Aabb3 {
        self.fat_aabb
    }
}

#[test]
fn test_frustum() {
    let mut tree = DynamicBoundingVolumeTree::<Value3>::new();
    tree.insert(Value3::new(10, aabb3(0.2, 0.2, 0.2, 10., 10., 10.)));
    tree.insert(Value3::new(11, aabb3(0.2, 0.2, -0.2, 10., 10., -10.)));
    tree.do_refit();

    let frustum = frustum();
    let mut visitor = FrustumVisitor::<Value3>::new(&frustum);
    let result = tree.query(&mut visitor);
    assert_eq!(1, result.len());
    let (v, r) = result[0];
    assert_eq!(Relation::Cross, r);
    assert_eq!(11, v.id);
}

fn aabb3(minx: f32, miny: f32, minz: f32, maxx: f32, maxy: f32, maxz: f32) -> Aabb3 {
    Aabb3::new(Vec3::new(minx, miny, minz), Vec3::new(maxx, maxy, maxz))
}

// Default perspective projection is looking down the negative z axis
fn frustum() -> Frustum {
    let projection = Mat4::perspective_rh_gl(60.0_f32.to_radians(), 16. / 9., 0.1, 4.0);
    projection.perspective_to_frustum()
}