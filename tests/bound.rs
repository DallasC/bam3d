extern crate bam3d;

use bam3d::PlaneBound;

fn _box(_: Box<dyn PlaneBound>) {}

#[test]
fn bound_box() {}