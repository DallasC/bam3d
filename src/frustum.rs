//! View frustum for visibility determination

use crate::plane::Plane;
use crate::bound::{PlaneBound, Relation};
use glam::{Mat4, Vec3};

/// View frustum, used for frustum culling
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Frustum {
    /// Left plane
    pub left: Plane,
    /// Right plane
    pub right: Plane,
    /// Bottom plane
    pub bottom: Plane,
    /// Top plane
    pub top: Plane,
    /// Near plane
    pub near: Plane,
    /// Far plane
    pub far: Plane,
}

impl Frustum {
    /// Construct a frustum.
    pub fn new(
        left: Plane,
        right: Plane,
        bottom: Plane,
        top: Plane,
        near: Plane,
        far: Plane,
    ) -> Frustum {
        Frustum {
            left,
            right,
            bottom,
            top,
            near,
            far,
        }
    }

    /// Extract frustum planes from a projection matrix.
    pub fn from_mat4(matrix: Mat4) -> Option<Frustum> {
        // to get rows instead of columns
        let mat = &matrix.transpose();
        Some(Frustum::new(
            match Plane::from_vector4_alt(mat.w_axis() + mat.x_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.w_axis() - mat.x_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.w_axis() + mat.y_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.w_axis() - mat.y_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.w_axis() + mat.x_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.w_axis() - mat.x_axis()).normalize() {
                Some(p) => p,
                None => return None,
            },
        ))
    }

    /// Find the spatial relation of a bound inside this frustum.
    pub fn contains<B: PlaneBound>(&self, bound: &B) -> Relation {
        [
            self.left,
            self.right,
            self.top,
            self.bottom,
            self.near,
            self.far,
        ].iter()
            .fold(Relation::In, |cur, p| {
                use std::cmp::max;
                let r = bound.relate_plane(*p);
                // If any of the planes are `Out`, the bound is outside.
                // Otherwise, if any are `Cross`, the bound is crossing.
                // Otherwise, the bound is fully inside.
                max(cur, r)
            })
    }
}

/// View frustum corner points
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FrustumPoints {
    /// Near top left point
    pub near_top_left: Vec3,
    /// Near top right point
    pub near_top_right: Vec3,
    /// Near bottom left point
    pub near_bottom_left: Vec3,
    /// Near bottom right point
    pub near_bottom_right: Vec3,
    /// Far top left point
    pub far_top_left: Vec3,
    /// Far top right point
    pub far_top_right: Vec3,
    /// Far bottom left point
    pub far_bottom_left: Vec3,
    /// Far bottom right point
    pub far_bottom_right: Vec3,
}

/// Conversion trait for converting glam projection matrixes into a view frustum
pub trait Projection: Into<Mat4> {
    /// Create a view frustum from a perspective projection
    fn perspective_to_frustum(&self) -> Frustum;
}

impl Projection for Mat4 {
    fn perspective_to_frustum(&self) -> Frustum {
        // TODO: Could this be faster?
        Frustum::from_mat4(*self).unwrap()
    }
}