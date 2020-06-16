//! Collision primitives

pub use self::capsule::Capsule;
pub use self::cuboid::{Cube, Cuboid};
pub use self::cylinder::Cylinder;
pub use self::particle::*;
pub use self::polyhedron::ConvexPolyhedron;
pub use self::primitive3::Primitive3;
pub use self::quad::Quad;
pub use self::sphere::Sphere;

mod cylinder;
mod capsule;
mod cuboid;
mod particle;
mod polyhedron;
mod primitive3;
mod quad;
mod sphere;

pub(crate) mod util;

use glam::{Vec3, Mat4};

pub use crate::bound::{PlaneBound, Relation};
pub use crate::traits::*;
pub use crate::volume::{Aabb, MinMax};

impl<B, P> HasBound for (P, B)
where
    P: Primitive,
    B: Bound,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.1
    }
}

impl<B, P> Primitive for (P, B)
where
    P: Primitive,
    B: Bound,
{
    fn support_point(
        &self,
        direction: &Vec3,
        transform: &Mat4,
    ) -> Vec3 {
        self.0.support_point(direction, transform)
    }
}