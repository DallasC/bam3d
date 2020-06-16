//! Generic spatial bounds.
use glam::{Vec3, Vec4, Mat4};
use std::{cmp, fmt};

use crate::plane::Plane;
use crate::frustum::Frustum;

/// Spatial relation between two objects.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialOrd, PartialEq)]
#[repr(u8)]
pub enum Relation {
    /// Completely inside.
    In,
    /// Crosses the boundary.
    Cross,
    /// Completely outside.
    Out,
}

/// Generic 3D bound.
pub trait PlaneBound: fmt::Debug {
    /// Classify the spatial relation with a plane.
    fn relate_plane(&self, plane: Plane) -> Relation;
    /// Classify the relation with a projection matrix.
    fn relate_clip_space(&self, projection: Mat4) -> Relation {
        let frustum = match Frustum::from_mat4(projection) {
            Some(f) => f,
            None => return Relation::Cross,
        };
        [
            frustum.left,
            frustum.right,
            frustum.top,
            frustum.bottom,
            frustum.near,
            frustum.far,
        ].iter()
            .fold(Relation::In, |cur, p| {
                let r = self.relate_plane(*p);
                // If any of the planes are `Out`, the bound is outside.
                // Otherwise, if any are `Cross`, the bound is crossing.
                // Otherwise, the bound is fully inside.
                cmp::max(cur, r)
            })
    }
}

impl PlaneBound for Vec3 {
    fn relate_plane(&self, plane: Plane) -> Relation {
        let dist = self.dot(plane.n);
        if dist > plane.d {
            Relation::In
        } else if dist < plane.d {
            Relation::Out
        } else {
            Relation::Cross
        }
    }

    fn relate_clip_space(&self, projection: Mat4) -> Relation {
        use std::cmp::Ordering::*;
        let p = projection * Mat4::from_cols(
            Vec4::new(1.0, 0.0, 0.0, self[0]),
            Vec4::new(0.0, 1.0, 0.0, self[1]),
            Vec4::new(0.0, 0.0, 1.0, self[2]),
            Vec4::new(0.0, 0.0, 0.0, 1.0)
        );
        match (
            p.x_axis().abs().partial_cmp(&p.w_axis()),
            p.y_axis().abs().partial_cmp(&p.w_axis()),
            p.z_axis().abs().partial_cmp(&p.w_axis()),
        ) {
            (Some(Less), Some(Less), Some(Less)) => Relation::In,
            (Some(Greater), _, _) | (_, Some(Greater), _) | (_, _, Some(Greater)) => Relation::Out,
            _ => Relation::Cross,
        }
    }

}