//! Expanding Polytope Algorithm

pub use self::epa3d::EPA3;

mod epa3d;

use glam::{Mat4};

use super::SupportPoint;
use crate::{contact::Contact, traits::Primitive};

pub const EPA_TOLERANCE: f32 = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

/// Expanding Polytope Algorithm base trait
pub trait EPA {
    /// Point type

    /// Process the given simplex, and compute the contact point.
    ///
    /// The given simplex must be a complete simplex for the given space, and it must enclose the
    /// origin.
    fn process<SL, SR>(
        &self,
        simplex: &mut Vec<SupportPoint>,
        left: &SL,
        left_transform: &Mat4,
        right: &SR,
        right_transform: &Mat4,
    ) -> Option<Contact>
    where
        SL: Primitive,
        SR: Primitive;

    /// Create a new EPA instance
    fn new() -> Self;

    /// Create a new EPA instance with the given tolerance
    fn new_with_tolerance(
        tolerance: f32,
        max_iterations: u32,
    ) -> Self;
}